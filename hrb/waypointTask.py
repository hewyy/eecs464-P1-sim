if __name__ != "__main__":
    raise RuntimeError("Run this as a script")

from socket import socket, AF_INET, SOCK_DGRAM, SOCK_STREAM, error as SocketError
from errno import EADDRINUSE
from sys import argv
from numpy import (
  array,asarray,zeros, exp,linspace, diff, ones_like, c_,
  pi, empty_like, nan, isnan, mean, dot, angle, asfarray
)
from gzip import open as gzip_open
from json import loads as json_loads, dumps as json_dumps
from time import time as now, sleep
from joy import JoyApp, speak, progress
from joy.plans import AnimatorPlan
from joy.decl import KEYDOWN
from waypointShared import (
    ref, fitHomography, Sensor, corners, waypoints, ROBOT_TAGID
)

#### CONFIGURATION ##################################################

# Host & Port for sending sensor readings
ROBOT_INET_ADDR = ("127.0.0.1",0xBAA)

# Rate (seconds) at which to send waypoint updates
# NOTE: waypoints are sent as integer coordinates. Set ref accordingly
WAY_RATE = 10
# Size of zoomed in robot region
ZOOM_SCALE = 10

## April tags associations ##########################################

# Port for receiving data from TagStreamer
APRIL_DATA_PORT = 0xB00

# EMA coefficient for static tag locations
alpha = 0.05

# Format of log filenames
logfmt = "log-%.0d-%s.txt.gz"
### ------------------------- CODE BEGINS --------------------------

# Clean up the socket if already open
try:
  s.close()
except Exception:
  pass
# Call this before socket is created so that child won't inherit socket
speak.say("")

def doVis(ax,mk,msg,prj = None):
    """
    Execute visualization message

    INPUTS:
      ax -- matplotlib axes -- axes to use for plotting
      mk -- str -- method name key to look for
      msg -- dict -- message as dictionary
      prj -- 3x3 -- homography; acting on the right; optional
    """
    # method name
    mn = msg.pop(mk)
    # '~' prefix indicates projective xform needed for arg[0], arg[1]
    doPrj = mn.startswith('~')
    if doPrj:
        mn = mn[1:]
    meth = getattr(ax,mn)
    # Collect positional arguments
    arg = []
    anm = ''
    while True:
        anm = '@%d' % len(arg)
        if not anm in msg:
            break
        arg.append(asarray(msg.pop(anm))/100.)
    # If projective correction requested
    if doPrj:
        assert len(arg)>1
        xy0 = c_[arg[0],arg[1],ones_like(arg[0])]
        xy1 = dot(xy0,prj)
        arg[:2] = (xy1[:,:2]/xy1[:,[2]]).T
    ##progress("VIS%s %s(*%s,**%s)" % (mk,mn,arg,msg))
    return meth(*arg,**msg)

###!!! from pdb import set_trace as BRK
SRV_PORT = 8080
for arg in argv:
    if arg.startswith("-p"):
        SRV_PORT = int(arg[2:])
def _animation(f1):
  global s, app
  # Open socket
  try:
    s = socket(AF_INET, SOCK_DGRAM )
    s.bind(("",APRIL_DATA_PORT))
    s.setblocking(0)
    # Server socket
    srv = socket(AF_INET, SOCK_STREAM )
    while True:
      try:
        srv.bind(("0.0.0.0",SRV_PORT))
        print("... listening at %s:%d" % srv.getsockname())
        break
      except SocketError as se:
        if se.errno == EADDRINUSE:
          print("... address in use. Waiting a bit ...")
          sleep(2)
    srv.listen(1)
  except:
    app.stop()
    raise
  client = None
  logfile = None
  # Axes for arena display
  ax = array([
      min(ref[:,0]),max(ref[:,0]),
      min(ref[:,1]),max(ref[:,1])
  ])*1.2
  # Allowed tag IDS
  allow = set(corners + waypoints + ROBOT_TAGID)
  # Array size; must store all allowed tags
  N = max(allow)+1
  # Array holding all point locations
  pts = zeros((N,4,3),dtype=float)
  # Indicator array for point that are assumed static
  #   and whose locations will be lowpass filtered
  statics = zeros(N,dtype=bool)
  statics[corners + waypoints] = True
  # Legend for point update indicator strings
  lbl = array(list(".+ld:*LD"))
  # (CONST) Reference locations for tag corners
  ang0 = array([-1+1j,1+1j,1-1j,-1-1j]) * -1j
  # (CONST) Point on a circle
  circ = exp(1j*linspace(0,2*pi,16))
  ### Initial values for variables
  # Configure sensor line-types
  sensorF = Sensor( ':om', lw=2 )
  sensorB = Sensor( ':oc', lw=2 )
  # Last message received from TagStreamer
  msg = None
  # Last waypoint visited
  M = 0
  # Dynamic zoom scale for robot view
  zoom = None
  # Last homography
  prj = None
  # Start time
  T0 = now()
  # Time last waypoint message was sent
  lastWay = T0-WAY_RATE-1
  # Number of messages received
  nmsg = 0
  #
  ### MAIN LOOP ###
  #
  while len(waypoints)>M+1: # continue until goal is reached
    #
    ### Read data from April
    #
    try:
      while True:
        # read data as fast as possible
        msg = s.recv(1<<16)
        nmsg += 1
    except SocketError:
      # until we've run out; last message remains in m
      pass
    # make sure we got something
    if not msg:
      continue
    # Parse tag information from UDP packet
    dat = [d for d in json_loads(msg) if type(d) is dict ]
    msg = ''
    # Parse tags and lines; collect tags
    lwor = []
    lrob = []
    h = empty_like(pts)
    h[:] = nan
    for d in dat:
        if '@r' in d: # Robot visualization lines
            lrob.append(d)
            continue
        if '@w' in d: # World visualization lines
            lwor.append(d)
            continue
        if 'i' in d:
            nm = d['i']
            if not nm in allow:
                continue
            #if ~isnan(h[nm,0,0]):
            #  print '(dup)',
            p = asfarray(d['p'])/100
            h[nm,:,:2] = p
            h[nm,:,2] = 1
            continue
        progress('Unknown dict in msg: %r' % d)
    #
    # at this point, all observed tag locations are in the dictionary h
    #
    ### Update pts array
    #
    # Tags seen in this frame
    fidx = ~isnan(h[:,0,0])
    # Tags previously unseen
    uidx = isnan(pts[:,0,0])
    # Tags to update directly: non static, or static and first time seen
    didx = (uidx & fidx) | ~statics
    if any(didx):
      pts[didx,...] = h[didx,...]
    # Tags to update with lowpass: static, seen and previously seen
    lidx = fidx & statics & ~uidx
    if any(lidx):
      pts[lidx,...] *= (1-alpha)
      pts[lidx,...] += alpha * h[lidx,...]
    # Print indicator telling operator what we did
    progress( "%7.2f %5d  "% (now()-T0,nmsg) + ''.join(lbl[didx+2*lidx+4*fidx])
             , sameLine = True )
    #
    # Collect the corner tags and estimate homography
    nprj = None
    try:
      roi = array( [ mean(pts[nmi],0) for nmi in corners ] )
      # Homography mapping roi to ref
      nprj = fitHomography( roi, ref )
    except KeyError as ck:
      progress("-- missing corner %s" % str(ck))
    #
    # If no previous homography --> try again
    if prj is None:
      # If no homography found
      if nprj is None:
        yield
        continue
      progress("(say) Homography initialized")
    prj = nprj
    #
    # Apply homography to all the points
    #progress(">>>!!! raw:" + repr(pts[ROBOT_TAGID]))
    uvs = dot(pts,prj)
    z = uvs[...,0] + 1j*uvs[...,1]
    nz = ~isnan(z[:,0])
    nz &= asarray(uvs[:,0,-1],dtype=bool)
    z[nz,...] /= uvs[nz,...,[-1]]
    # Centroids of tags
    zc = mean(z,1)
    #progress(">>>!!! zc:" + repr(zc[ROBOT_TAGID]))
    #
    ### At this point, z has all tag corner points; zc centroids
    ###   the nz index indicated tags for which we have locations
    #
    f1.clf()
    a1 = f1.add_subplot(121)
    # Mark the corner tags
    c = zc[corners]
    vc = ~isnan(c)
    a1.plot( c[vc].real, c[vc].imag,'sy',ms=15)
    # Indicate all tags
    znz = z[nz,...]
    assert ~any(isnan(znz).flatten())
    a1.plot( znz[:,[0,1,2,3,0]].real.T, znz[:,[0,1,2,3,0]].imag.T, '.-b' )
    a1.plot( [znz[:,0].real], [znz[:,0].imag], 'og' )
    for nm,p in enumerate(zc):
      if not isnan(p):
        a1.text( p.real,p.imag, "%d" % nm, ha='center',va='center' )
    # Mark the waypoints
    c = zc[waypoints]
    vc = ~isnan(c)
    a1.plot( c[vc].real, c[vc].imag,'-k',lw=3,alpha=0.3)
    if not any(isnan(c[[M,M+1]].real)):
      a1.plot( c[[M,M+1]].real, c[[M,M+1]].imag, '--r', lw=4)
    # Client visualization
    for ln in lwor:
        try:
            doVis(a1,'@w',ln,prj)
        except Exception as ex:
            progress("World vis error: %s \n\t\tFrom: %r" % (ex,ln))
    a1.axis('equal')
    a1.axis(ax)
    #
    # If robot not seen --> we're done
    if isnan(zc[ROBOT_TAGID]):
      yield
      continue
    #
    ### robot tag related updates
    #
    # robot tag corners
    rbt = z[ROBOT_TAGID,...]
    # robot heading angle phasor
    ang = mean((rbt-zc[ROBOT_TAGID])/ang0)
    ang /= abs(ang)
    # If logging data --> put into log
    if logfile is not None:
      lwp = len(waypoints)-1
      lo = [ now(), zc[ROBOT_TAGID].real, zc[ROBOT_TAGID].imag, angle(ang),
             int(zc[waypoints[M]].real), int(zc[waypoints[M]].imag),
             int(zc[waypoints[min(M+1,lwp)]].real), int(zc[waypoints[min(M+1,lwp)]].imag) ]
      logfile.write((", ".join(["%.3f" % x for x in lo])+"\n").encode('ascii'))
    # indicate robot
    a1.plot( zc[ROBOT_TAGID].real, zc[ROBOT_TAGID].imag, '*r' ,ms=15)
    #
    ### robot relative view
    #
    a2 = f1.add_subplot(122)
    #
    # Show the waypoints
    c = zc[waypoints]
    vc = ~isnan(c)
    cr = (c - zc[ROBOT_TAGID])/ang
    a2.plot( cr[vc].real, cr[vc].imag,'-k',lw=3,alpha=0.3)
    if not any(isnan(cr[[M,M+1]].real)):
      a2.plot( cr[[M,M+1]].real, cr[[M,M+1]].imag, '--r', lw=4)
    #
    # Show the tags
    zrr = (z - zc[ROBOT_TAGID])/ang
    znr = zrr[nz,...]
    a2.plot( znr[:,[0,1,2,3,0]].real.T, znr[:,[0,1,2,3,0]].imag.T, '.-b' )
    a2.plot( [znr[:,0].real], [znr[:,0].imag], 'og' )
    for nm,p in enumerate(zrr):
      if not isnan(p[0]) and nz[nm]:
        q = mean(p)
        a2.text( q.real,q.imag, "%d" % nm, ha='center',va='center' )
    #
    ### Check for waypoint contact
    #
    # Tag scale
    r = max(abs(diff(zrr[ROBOT_TAGID].flat)))
    ###!!! progress("\n:::"+repr(r))
    # Indicate goal circle
    tgt = circ*r
    tgth = [
      a2.plot( tgt.real, tgt.imag, '-k' )[0],
      a2.plot( tgt.real/1.5, tgt.imag/1.5, '--k' )[0]
    ]
    # Update dynamic zoom
    if zoom is not None:
      zoom = zoom * 0.9 + r * 0.1
    else:
      zoom = r
    #
    # Show client visualization, if any
    if lrob:
        rprj = dot(prj,dot(
            [   [1,0,0],
                [0,1,0],
                [-zc[ROBOT_TAGID].real,-zc[ROBOT_TAGID].imag,1]],
            [   [ ang.real,-ang.imag,0],
                [ ang.imag, ang.real,0],
                [0,0,1]]))
        for ln in lrob:
            try:
                doVis(a2,'@r',ln,rprj)
            except Exception as ex:
                progress("Robot vis error: %s \n\t\tFrom: %r" % (ex,ln))
    # Check distance of both sensors to the line
    a,b = cr[[M,M+1]]
    # Build into packet
    pkt = {
      'f' : float(sensorF.sense( a2, a, b, zrr[ROBOT_TAGID,0]+zrr[ROBOT_TAGID,1], r )),
      'b' : float(sensorB.sense( a2, a, b, zrr[ROBOT_TAGID,2]+zrr[ROBOT_TAGID,3], r )),
    }
    # Check for waypoint capture
    if abs(b)<zoom:
      for h in tgth:
        h.set_linewidth(3)
        h.set_color([0,0,1])
      progress("(say) Reached waypoint %d" % (M+1))
      lastWay = now()-WAY_RATE-1
      M += 1
      if (M+1)>=len(waypoints):
        app.stop()
        return
    # Add waypoint update to packet, as needed
    if now()-lastWay>WAY_RATE:
      pkt['w'] = list(zip(
        [ int(x) for x in zc[waypoints[M:]].real],
        [ int(x) for x in zc[waypoints[M:]].imag]
      ))
      lastWay = now()
    # If we don't have a client -- listen
    if client is None:
      progress( "Waiting for client connection" )
      client,addr = srv.accept()
      progress( "Connection from "+str(addr) )
      if logfmt:
        logfile = gzip_open(logfmt % (now(),addr[0]),'w')
    # Send sensor reading out
    try:
      client.send( json_dumps(pkt).encode('ascii') )
      progress( "%s %s       " % (pkt['b'],pkt['f']),sameLine=True)
    except SocketError as er:
      progress( str(er) )
      progress( "Connection dropped" )
      if logfile:
        logfile.close()
        logfile = None
      client = None
    #
    a2.axis('equal')
    a2.axis( array([-1,1,-1,1])*ZOOM_SCALE*zoom)
    yield


class App(JoyApp):
  def __init__(self,*arg,**kw):
      if 'cfg' not in kw:
          kw['cfg'] = dict(remote=False)
      else:
          kw['cfg'].update(remote=False)
      return JoyApp.__init__(self,*arg,**kw)

  def onStart(self):
    AnimatorPlan(self,_animation).start()

  def onEvent(self,evt):
      if evt.type == KEYDOWN:
        return JoyApp.onEvent(self,evt)

if __name__=="__main__":
  app = App(cfg={'windowSize' : [1200,600]})
  app.run()

# TODO: auto activation of subprocess tagstreamer
# TODO: Cleanup into class format
# TODO: yaml / json for arena config
