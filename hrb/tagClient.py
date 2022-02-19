from joy import JoyApp, progress
from joy.plans import AnimatorPlan
from joy.decl import KEYDOWN 

from socket import socket, AF_INET, SOCK_DGRAM, error as SocketError
from numpy import array, mean
from json import loads as json_loads
from time import time as now

if __name__ != "__main__":
  raise RuntimeError("Run this as a script")

try:
  s.close()
except Exception:
  pass

def _animation(fig):
  s = socket(AF_INET, SOCK_DGRAM )
  s.bind(("",0xB00))
  s.setblocking(0)
  fig.clf()
  ax = fig.add_subplot(111)
  msg = None
  src = None
  last = now()
  while True:
    try:
      # read data as fast as possible
      m,msrc = s.recvfrom(1<<16)
      if not (msrc == src):
        src = msrc
        progress("New tag data source: %s:%d"%src)
      if m and len(m)>2:
        msg = m
      continue
    except SocketError as se:
      # until we've run out; last message remains in m
      pass
    # make sure we got something
    if not msg:
      yield
      continue
    # display it
    ax.cla()
    ax.set_title("%.1f fps" % (1/(now()-last)))
    last = now() 
    for d in json_loads(msg):
      if type(d) is not dict:
        continue
      a = array(d['p'])
      ax.plot( a[[0,1,2,3,0],0], a[[0,1,2,3,0],1], '.-b' )
      ax.plot( a[[0],0], a[[0],1], 'og' )
      ax.text( mean(a[:,0]), mean(a[:,1]), d['i'], ha='center',va='center' )
      ax.axis([0,1600,0,1200])
    yield
    
      
class App(JoyApp):
  def onStart(self):
      AnimatorPlan(self,_animation).start()

  def onEvent(self,evt):
      if evt.type == KEYDOWN:
        return JoyApp.onEvent(self,evt)
        
app = App(cfg={'windowSize':(1080,740)})
app.run()
  
