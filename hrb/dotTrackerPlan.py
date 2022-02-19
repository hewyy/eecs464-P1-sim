from numpy import asarray,zeros,mean,std
from gzip import open as opengz
from time import time as now
from joy.decl import *
from joy.plans import Plan
from joy.remote import Source

assert False, "Needs upgrade to use CV2; base on laserPointerTracker.py"

from SimpleCV import *

class DotTrackerPlan( Plan ):
  def __init__(self, app,*argv, **kw):
    Plan.__init__(self,app,*argv,**kw)

  def onStart(self):
    self.blobs = []
    self.depth = 5 # seconds
    self.cam = JpegStreamCamera("http://admin:hrb2017@35.2.254.129:8081/video")
    self.win = Display((800,600))
    self.cam.getImage().save(self.win)
    self.clearBG()
  def clearBG( self ):
    self.bgN = 5
    self.bgQ = None
    self.bg = None
    self.tsQ = []
    self.blob = []
    self.qi = -1
    self.trr = 0
    self.rrRate = 2
    self.guard = 10

  def _putRoundRobin( self, ts, img ):
    qi = (self.qi+1) % self.bgN
    if self.bgQ is None:
      self.bgQ = zeros( (self.bgN,)+img.shape, int )
      self.tsQ = zeros( (self.bgN,), int )
    # arrays are ready
    self.bgQ[qi,...] = img
    self.tsQ[qi] = ts
    self.qi = qi
    self.trr = ts

  def _outBlobs( self, t, b ):
    if b is None:
      self.blobs.append(dict(time=self.app.now,tsense=t,b=[]))
    else:
      self.blobs.append(dict(time=self.app.now,tsense=t,b=asarray(b.coordinates())))
    # Make sure that entries older than depth are removed
    while self.blobs and self.blobs[0]['time']<self.app.now-self.depth:
      self.blobs.pop(0)

  def _showBlobs( self, b ):
    raw = self.raw
    if b is not None:
      dl = raw.dl()
      for xy in b.coordinates():
        dl.circle(xy,10,Color.RED,filled=1)
        dl.circle(xy,5,Color.BLACK,filled=1)
    raw.save(self.win)

  def work( self ):
    raw = self.cam.getImage()
    self.raw = raw
    self.raw.save(self.win)
#    self.clearBG()
    img = raw.getGrayNumpy()
    self.img = img
    t = now()
    if self.bg is not None:
      ind = img > (self.bg+3*self.bgs+self.guard)
      ind = ind & (img > 240)
      self.ind = ind
      b = Image(ind).findBlobs()
      self._outBlobs(t,b)
      self._showBlobs(b)
    if t-self.trr>self.rrRate:
      self._putRoundRobin( t, img )
      self.bg = np.mean( self.bgQ, axis=0 )
      self.bgs = np.std( self.bgQ, axis=0 )

  def behavior( self ):
    #self.cam.getImage().save(self.win)
    try:
      while not self.win.isDone():
        self.work()
#	print(self.win.isDone())
        yield self.forDuration(0.05)
    except KeyboardInterrupt:
      self.win.quit()
      print("\n"+"*"*40+"\nSafely terminated\n"+"*"*40)
      raise


class DotTrackerSource(Source,DotTrackerPlan):
  def __init__(self,app,*argv,**kw):
    Source.__init__(self,app,*argv,**kw)

  def onStart(self):
    Source.onStart(self)
    return DotTrackerPlan.onStart(self)

  def onStop(self):
    DotTrackerPlan.onStop()
    return Source.onStop()

  def onEvent(self,evt):
    return True

  def _outBlobs( self, t, b ):
    if b is None:
      return
    msg = dict(
      time=self.app.now,
      tsense=t,
      b=[ [x,y] for (x,y) in b.coordinates()]
      )
    self.sendMsg(**msg)

if __name__=="__main__":
   from joy import JoyApp
   class App(JoyApp):
     def onStart(self):
       self.dt = DotTrackerSource(self)
       self.dt.start()
       self.oe = self.onceEvery(1)

     def onEvent(self,evt):
       if self.oe():
         progress("OE:" + str(self.dt.blobs[-1:]))
       # Punt keydown events to superclass
       if evt.type == KEYDOWN:
         return JoyApp.onEvent(evt)

   App().run()
