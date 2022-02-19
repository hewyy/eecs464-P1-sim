
from gzip import open as opengz
from time import time as now
from pdb import set_trace as BRK

# Standard imports
from cv2 import (
    SimpleBlobDetector_create as SimpleBlobDetector, VideoCapture, drawKeypoints,
    imshow as cv2_imshow, imwrite as cv2_imwrite,
    COLOR_RGB2GRAY, cvtColor, circle as cv2_circle,
    destroyAllWindows, waitKey, SimpleBlobDetector_Params
    )
from numpy import (
  zeros, int16, uint8, asarray, mean, std
)

class LaserTracker( object ):
  def __init__(self, fn=None, cam=0):
    bdp = SimpleBlobDetector_Params()
    bdp.minArea = 5
    bdp.maxArea = 300
    bdp.filterByInertia = False
    bdp.filterByConvexity = False
    self.bd = SimpleBlobDetector(bdp)
    self.cam = VideoCapture(cam)
    #self.cam = VideoCapture("http://admin:hrb2018@172.18.18.3:8080/video")
    self.clearBG()
    if not fn:
      self.out = None
    else:
      self.out = opengz(fn,"w")
      ok,img = self.cam.read()
      cv2_imwrite(fn+'first.png',img)

  def clearBG( self ):
    self.bgN = 10
    self.bgQ = None
    self.bg = None
    self.tsQ = []
    self.blob = []
    self.qi = -1
    self.trr = 0
    self.rrRate = 0.5
    self.guard = 10

  def _putRoundRobin( self, ts, img ):
    qi = (self.qi+1) % self.bgN
    if self.bgQ is None:
      self.bgQ = zeros( (self.bgN,)+img.shape, int16 )
      self.tsQ = zeros( (self.bgN,), int16 )
    # arrays are ready
    self.bgQ[qi,...] = img
    self.tsQ[qi] = ts
    self.qi = qi
    self.trr = ts

  def _outBlobs( self, t, b ):
    if not b:
      self.out.write(b"%.2f, 0, 0, 0\n" % t)
    else:
      for n,(x,y) in enumerate(b):
        self.out.write(b"%.2f, %d, %d, %d\n" % (t,n+1,x,y))
    self.out.flush()

  def _showBlobs( self, b ):
    img = self.raw.copy()
    if b is not None:
      for x,y in b:
        xy = (int(x),int(y))
        cv2_circle(img,xy,10,(0,255,0),thickness=2)
        #cv2_circle(img,xy,5,(0,0,0),thickness=-1)
    cv2_imshow('Tracker',img)

  def work( self ):
    res,raw = self.cam.read()
    if not res:
        return
    self.raw = raw
    img = cvtColor(raw,COLOR_RGB2GRAY)
    self.img = img
    t = now()
    if self.bg is not None:
      d0 = img - (self.bg+3*self.bgs+self.guard)
      d1 = (d0 / max(64,d0.max())).clip(0,1)
      d2 = d1 * (img > 192)
      self.ind = 255-asarray(255*d2,uint8)
      cv2_imshow('ind',self.ind)
      kp = self.bd.detect(self.ind)
      b = [kpi.pt for kpi in kp]
      if self.out:
        self._outBlobs(t,b)
      self._showBlobs(b)
    if t-self.trr>self.rrRate:
      self._putRoundRobin( t, img )
      self.bg = mean( self.bgQ, axis=0 )
      self.bgs = std( self.bgQ, axis=0 )

  def run( self ):
    res,img = self.cam.read()
    if res:
        cv2_imshow('Tracker',img)
    try:
      while waitKey(5) not in {27,113}:
        self.work()
    finally:
      self.out.close()
      self.cam.release()
      destroyAllWindows()
      [ waitKey(50) for k in range(10) ]
      print("\n"+"*"*40+"\nSafely terminated\n"+"*"*40)


if __name__=="__main__":
  import sys
  if len(sys.argv) < 2:
    sys.stderr.write("Usage: %s <filename>\n" % sys.argv[0])
    fn = 'foo.csv.gz'
  else:
    fn = sys.argv[1]
    if not fn.endswith('.gz'):
      fn = fn + ".gz"
  sys.stderr.write("  output to %s\n" % fn)
  if len(sys.argv) > 2:
    cam = int(sys.argv[2])
  else:
    cam = 0
  lt = LaserTracker(fn,cam)
  lt.run()
