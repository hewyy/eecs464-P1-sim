
from scipy.linalg import norm, pinv
from numpy import (
  sqrt, asarray, asfarray, concatenate, ones, ones_like, c_, dot, zeros )
from matplotlib.pyplot import (
  subplot, show, gcf, gca
  )
from arm import Arm, jacobian_cdas
from cmd import Cmd

class Camera(object):
  """
  Define a projective camera

  Takes homogeneous 3D points and returns pixel coordinates
  """
  def __init__(self, C=None):
    if C is None:
      C = zeros((3,4),float)
      C[[0,1,2],[0,1,2]]=1,1,1
    else:
      C = asfarray(C)
      assert C.shape == (3,4)
    self.cam = C

  def project(self,pts):
    pts = asfarray(pts)
    assert pts.shape[0] == 4
    if pts.ndim == 1:
      pts.shape = 4,1
    uvs = dot(self.cam,pts)
    return uvs[:2,:]/uvs[[2],:]

  def __call__(self,pts):
    return self.project(pts)

class VisCtrl( object ):
  def __init__(self, arm):
    # Multiple camera views
    self.cams = [
      Camera([
        [1,1,0,0],
        [0,0,1.57,0],
        [.1,-.1,0,-10]
        ]),
      Camera([
        [1,-1,0,0],
        [0,0,1.57,0],
        [.1,.1,0,-10]
        ]),
    ]
    self.arm = arm
    self.goal = asarray([1,1,1,1])
    # overwrite method with jacobian function
    self.toolCamJac = jacobian_cdas(
      self.getToolPix, ones(self.arm.tw.shape[0])*0.05
    )

  def getPixels(self, pts):
    """
    Convert points to pixels in all cameras
    """
    return concatenate( [cam(pts) for cam in self.cams], axis=0 ).squeeze()[:3]

  def getToolPix( self, ang ):
    """
    Get tool pixel coordinates
    """
    return self.getPixels( self.arm.getTool(ang) )

  def ctrlStep( self, ang, scl = 0.1 ):
    """
    Make one visual servoing update, with actual step size restricted by scl
    """
    Jt = self.toolCamJac(ang)
    g = self.getPixels(self.goal)
    t = self.getToolPix(ang)
    iJt = pinv(Jt)
    d = dot(iJt[:,:3],(g-t)[:3])
    l = norm(d)
    if l>scl:
      return (scl * d / norm(d))
    else:
      return d

  def plot3D(self, x, y, z, *arg, **kw):
      pts = c_[x,y,z,ones_like(z)].T
      N = len(self.cams)
      m = int(sqrt(N)+0.5)
      for k,cam in enumerate(self.cams):
          subplot(m,m+1,k+1)
          v,u = cam(pts)
          ax = gca()
          ax.plot(v,u,*arg,**kw)
          ax.axis('equal')
          ax.grid(1)
          ax.set_title("Camera %d" % k)
          
  def plotAll( self, ang ):
    A = self.arm.at(ang)
    self.arm.plot3D( A, ax=self )
    g = self.goal
    self.plot3D( g[0], g[1], g[2], 'xr', mew=2, ms=15)
    self.plot3D( g[0], g[1], g[2], '+r', mew=2, ms=15)
    self.plot3D( [-10,10],[-10,10],[-10,10], marker='.', color='w', alpha=0.1)

class VisCtrlExample(Cmd):
  def __init__(self):
    Cmd.__init__(self)
    self.arm = Arm(asarray([[0,1,0,3],[0,0,1,3]]*3).T)
    self.vc = VisCtrl(self.arm)
    self.ang = [0]*len(self.arm)
    self._vis = self._vis3D

  def postcmd( self, stop, line):
    if not stop:
      self._vis()
    return stop

  def _vis3D(self):
    f = gcf()
    f.clf()
    self.arm.plotAll(self.ang)
    show()
    print("Angles: ",self.ang)

  def _visCams(self):
    f = gcf()
    f.clf()
    self.vc.plotAll(self.ang)
    show()
    print("Angles: ",self.ang)

  def do_quit(self,line):
    """quit"""
    return True

  def do_EOF(self,line):
    """quit"""
    return True

  def do_viscams(self,line):
    """
    Visualize camera views
    """
    print("Visualizing cameras")
    self._vis = self._visCams

  def do_vis3d(self,line):
    """
    Visualize 3D orthographic views (without goal)
    """
    print("Visualizing 3D standard views")
    self._vis = self._vis3D

  def do_ang(self,line):
    """
    Set joint angles
    """
    lst = eval('['+line+']')
    if len(lst)>len(self.ang):
        print(("ERROR: too many angles; need %d got %d" % (len(self.ang),len(lst))))
        return
    self.ang[:len(lst)] = lst
    print("Setting angles to:",lst)

  def do_goal(self,line):
    """
    Set goal position for controller
    """
    lst = eval('['+line+']')
    if len(lst) != 3:
      print("Goal position must be 3D point")
      return
    self.vc.goal[:3] = lst
    print("Goal set to:",lst)

  def do_move(self,line):
    """
    Move in specified direction
    """
    lst = eval('['+line+']')
    if len(lst) != 3:
      print("Direction must be 3D vector")
      return
    Jt = self.arm.getToolJac(self.ang)
    self.ang = self.ang + dot(pinv(Jt)[:,:3],lst)

  def do_run(self,line):
    """
    Run controller until goal. Optionally, specify maximal step size
    in joint-space (default = 0.1)
    """
    if line.strip():
      scl = float(line.strip())
    else:
      scl = 0.1
    f = gcf()
    dang = [scl,scl]
    ang = self.ang
    while norm(dang)>0.1*scl:
      f.clf()
      self.vc.plotAll(ang)
      show()
      print("Angles: ",ang)
      input("<<hit enter>>")
      dang = self.vc.ctrlStep(ang,scl)
      ang = ang + dang
    self.ang = ang

if __name__=="__main__":
  app = VisCtrlExample()
  app.cmdloop()
