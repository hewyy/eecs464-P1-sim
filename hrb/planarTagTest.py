if __name__ != "__main__":
  raise RuntimeError("Run this as a script")

from joy import JoyApp, progress
from joy.plans import AnimatorPlan
from joy.decl import KEYDOWN
from numpy import asarray,zeros_like,kron,concatenate,array,mean,dot,angle,pi,c_
from numpy.linalg import svd
from json import loads as json_loads
from sys import stdout
from socket import socket, AF_INET, SOCK_DGRAM, error as SocketError

try:
  s.close()
except Exception:
  pass

def skew( v ):
  """
  Convert a 3-vector to a skew matrix such that 
    dot(skew(x),y) = cross(x,y)
  
  The function is vectorized, such that:
  INPUT:
    v -- N... x 3 -- input vectors
  OUTPUT:
    N... x 3 x 3  
    
  For example:
  >>> skew([[1,2,3],[0,0,1]])
  array([[[ 0,  3, -2],
        [-3,  0,  1],
        [ 2, -1,  0]],
  <BLANKLINE>
       [[ 0,  1,  0],
        [-1,  0,  0],
        [ 0,  0,  0]]])
  """
  v = asarray(v).T
  z = zeros_like(v[0,...])
  return array([
      [ z, -v[2,...], v[1,...]],
      [v[2,...], z, -v[0,...] ],
      [-v[1,...], v[0,...], z ] ]).T

def fitHomography( x, y ):
  """Fit a homography mapping points x to points y"""
  x = asarray(x)
  assert x.shape == (len(x),3)
  y = asarray(y)
  assert y.shape == (len(y),3)
  S = skew(y)
  plan = [ kron(s,xi) for s,xi in zip(S,x) ]
  #plan.append([[0]*8+[1]])
  A = concatenate( plan, axis=0 )
  U,s,V = svd(A)
  res = V[-1,:].reshape(3,3)
  return res.T

def _animation(fig):
  global s

  s = socket(AF_INET, SOCK_DGRAM )
  s.bind(("",0xB00))
  s.setblocking(0)
  
  lst = []
  msg = None
  rh={}
  i=0 	
  
  # Corners of the arena, in order
  corners = [26,23,27,22,29,24,28,25]
  ref = array([
    [-1,0,1,1,1,0,-1,-1],
    [1,1,1,0,-1,-1,-1,0],
    [1.0/100]*8]).T * 100
  
  ax = array([
      min(ref[:,0]),max(ref[:,0]),
      min(ref[:,1]),max(ref[:,1])
  ])*1.2
  
  allow = set(corners + [12,14,13,15])
  fr = 0
  while True: #number of samples
    try:
      while True:
        # read data as fast as possible
        msg = s.recv(1<<16)
    except SocketError as se:
      # until we've run out; last message remains in m
      pass
    # make sure we got something
    if not msg:
      yield
      continue
    # Parse tag information from UDP packet
    dat = json_loads(msg)
    # Make sure there are enough tags to make sense
    if len(dat)<5:
      yield
      continue  
    # Collect allowed tags
    c = {}
    h = {}
    acc = []
    for d in dat:
      nm = d['i']
      if not nm in allow:
        continue
      p = asarray(d['p'])/100
      c[nm] = mean(p,0)
      h[nm] = p 
      acc.append(str(nm))
    # Collect the corner tags
    try:
      roi = array( [ [c[nm][0], c[nm][1], 1] for nm in corners ] )
    except KeyError as ck:
      progress( "-- missing corner %d" % ck)
      yield
      continue
    progress(",".join(acc))
    # Homography mapping roi to ref
    prj = fitHomography( roi, ref )
    mrk = dot(roi,prj)
    mrk = mrk[:,:2] / mrk[:,[-1]]
    print()
    # display it
    fig.clf()
    fa = fig.gca()
    # Mark the corner tags
    fa.plot( mrk[:,0],mrk[:,1],'sc',ms=15)
    ang0 = [-1+1j,1+1j,1-1j,-1-1j]
    # Loop on all tags
    for nm,p in h.items():
      # Project back
      a = dot(c_[p,[1]*len(p)],prj)
      a = a[:,:2]/a[:,[-1]]
      # Compute position
      z = a[:,0]+1j*a[:,1]
      mz = mean(z)
      # Compute angle
      ang = angle(mean((z-mz) / ang0))
      fa.plot( z[[0,1,2,3,0]].real, z[[0,1,2,3,0]].imag, '.-b' )
      fa.plot( [z[0].real], [z[0].imag], 'og' )
      fa.text( mean(a[:,0]), mean(a[:,1]), 
        "[%d] \n (%3d,%3d) %.0f" % (nm,mz.real,mz.imag,180/pi*ang),
        ha='center',va='center' )
      fa.axis(ax)
    #
    yield


class App(JoyApp):
  def onStart(self):
    AnimatorPlan(self,_animation).start()

  def onEvent(self,evt):
      if evt.type == KEYDOWN:
        return JoyApp.onEvent(self,evt)
        
app = App()
app.run()
