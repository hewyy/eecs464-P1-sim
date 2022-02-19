from mpl_toolkits.mplot3d import Axes3D
from scipy.linalg import expm as expM
from numpy import (
  asarray, all, empty, empty_like, concatenate, cross, dot,
  ones, newaxis, identity, zeros_like, array, diag, sum
)
from pylab import (
  gcf, plot, axis, clf, subplot, grid, xlabel, ylabel
)
def seToSE( x ):
  """
  Convert a twist (a rigid velocity, element of se(3)) to a rigid
  motion (an element of SE(3))

  INPUT:
    x -- 6 sequence
  OUTPUT:
    result -- 4 x 4

  """
  x = asarray(x,dtype=float)
  if x.shape != (6,):
    raise ValueError("shape must be (6,); got %s" % str(x.shape))
  #
  return expM(screw(x))

def screw( v ):
  """
  Convert a 6-vector to a screw matrix

  The function is vectorized, such that:
  INPUT:
    v -- N... x 6 -- input vectors
  OUTPUT:
    N... x 4 x 4
  """
  v = asarray(v)
  z = zeros_like(v[0,...])
  return array([
      [ z, -v[...,5], v[...,4], v[...,0] ],
      [ v[...,5],  z,-v[...,3], v[...,1] ],
      [-v[...,4],  v[...,3], z, v[...,2] ],
      [ z,         z,        z, z] ])

def jacobian_cdas( func, scl, lint=0.8, tol=1e-12, eps = 1e-30, withScl = False ):
  """Compute Jacobian of a function based on auto-scaled central differences.

  INPUTS:
    func -- callable -- K-vector valued function of a D-dimensional vector
    scl -- D -- vector of maximal scales allowed for central differences
    lint -- float -- linearity threshold, in range 0 to 1. 0 disables
         auto-scaling; 1 requires completely linear behavior from func
    tol -- float -- minimal step allowed
    eps -- float -- infinitesimal; must be much smaller than smallest change in
         func over a change of tol in the domain.
    withScl -- bool -- return scales together with Jacobian

  OUTPUTS: jacobian function
    jFun: x --> J (for withScale=False)
    jFun: x --> J,s (for withScale=True)

    x -- D -- input point
    J -- K x D -- Jacobian of func at x
    s -- D -- scales at which Jacobian holds around x
  """
  scl = abs(asarray(scl).flatten())
  N = len(scl)
  lint = abs(lint)
  def centDiffJacAutoScl( arg ):
    """
    Algorithm: use the value of the function at the center point
      to test linearity of the function. Linearity is tested by
      taking dy+ and dy- for each dx, and ensuring that they
      satisfy lint<|dy+|/|dy-|<1/lint
    """
    x0 = asarray(arg).flatten()
    y0 = func(x0)
    s = scl.copy()
    #print "Jac at ",x0
    idx = slice(None)
    dyp = empty((len(s),len(y0)),x0.dtype)
    dyn = empty_like(dyp)
    while True:
      #print "Jac iter ",s
      d0 = diag(s)
      dyp[idx,:] = [ func(x0+dx)-y0 for dx in d0[idx,:] ]
      dypc = dyp.conj()
      dyn[idx,:] = [ func(x0-dx)-y0 for dx in d0[idx,:] ]
      dync = dyn.conj()
      dp = sum(dyp * dypc,axis=1)
      dn = sum(dyn * dync,axis=1)
      nul = (dp == 0) | (dn == 0)
      if any(nul):
        s[nul] *= 1.5
        continue
      rat = dp/(dn+eps)
      nl = ((rat<lint) | (rat>(1.0/lint)))
      # If no linearity violations found --> done
      if ~any(nl):
        break
      # otherwise -- decrease steps
      idx, = nl.flatten().nonzero()
      s[idx] *= 0.75
      # Don't allow steps smaller than tol
      s[idx[s[idx]<tol]] = tol
      if all(s[idx]<tol):
        break
    res = ((dyp-dyn)/(2*s[:,newaxis])).T
    if withScl:
      return res, s
    return res
  return centDiffJacAutoScl

class Arm( object ):
  """
  class Arm

  Represents a series manipulator made of several segments.
  Each segment is graphically represented by a wireframe model

  ATTRIBUTES:
    tw --
  """
  def __init__(self,wl):
    return self.setup(wl)
  
  def setup(self, wl):
    self.ll = wl[3]
    # arm geometry to draw
    d=0.2
    hexa = asarray([
        [ 0, d,1-d, 1, 1-d, d, 0],
        [ 0, 1,  1, 0,  -1,-1, 0],
        [ 0, 0,  0, 0,   0, 0, 0],
        [ 1, 1,  1, 1,   1, 1, 1],
    ]).T
    sqr = asarray([
        [ d, d, d, d, d, 1-d, 1-d, 1-d, 1-d, 1-d],
        [ 1, 0,-1, 0, 1, 1, 0,-1, 0, 1 ],
        [ 0, 1, 0,-1, 0, 0, 1, 0,-1, 0],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 ],
    ]).T
    geom = concatenate([
      hexa, hexa[:,[0,2,1,3]], sqr,
    ], axis=0)
    self.geom = [( asarray([[0,0,0,1]]) ).T ]
    #
    # Build twist matrices
    # Build wireframe of all segments in the world coordinates
    #
    tw = []
    LL = 0
    for n,ll in enumerate(self.ll):
      # Scale the geometry to the specifies link length (ll)
      # Shift it to the correct location (LL, sum of the ll values)
      gn = ( asarray([ll,1,1,1])*geom+[LL,0,0,0] ).T
      self.geom.append(gn)
      # Compute the twist for this segment; first get rotation axis
      w = wl[:3,n]
      # Velocity induced at the origin
      v = -cross(w,[LL,0,0])
      # Collect the twists
      tw.append( concatenate([v,w],0) )
      # Accumulate the distance along the arm
      LL += ll
    # Build an array of collected twists
    self.tw = asarray(tw)
    # Position of tool
    self.tool = asarray([LL,0,0,1]).T
    # overwrite method with jacobian function
    self.getToolJac = jacobian_cdas(
      self.getTool, ones(self.tw.shape[0])*0.05
    )

  def at( self, ang ):
    """
    Compute the rigid transformations for a multi-segment arm
    at the specified angles
    """
    ang = asarray(ang)[:,newaxis]
    tw = ang * self.tw
    A = [identity(4)]
    for twi in tw:
      M = seToSE(twi)
      A.append(dot(A[-1],M))
    return A

  def __len__(self):
    return len(self.tw)
  
  def getTool( self, ang ):
    """
    Get "tool tip" position in world coordinates
    """
    # Get the rigid transformation for the last segment of the arm
    M = self.at(ang)[-1]
    return dot(M, self.tool)

  def getToolJac( self, ang ):
    """
    Get "tool tip" Jacobian by numerical approximation

    NOTE: implementation is a placeholder. This method is overwritten
    dynamically by __init__() to point to a jacobian_cdas() function
    """
    raise RuntimeError("uninitialized method called")

  def getSkel( self, A, withTool = True):
    """
    Get 'skeleton' of arm -- only the points connecting the segments
    """
    return asarray([ dot(a,g[:,0]) for a,g in zip(A, self.geom) ]
                    + ([] if not withTool else [dot(A[-1],self.tool)]))
    
  def plotIJ( self, A, axI=0, axJ=1 ):
    """
    Display the specified axes of the arm at the specified set of angles
    """
    for a,g in zip(A, self.geom):
      ng = dot(a,g)
      plot( ng[axI,:], ng[axJ,:], '.-' )
    tp = dot(a, self.tool)
    plot( tp[axI], tp[axJ], 'hk' )
    plot( tp[axI], tp[axJ], '.y' )

  def plot3D(self, A, ax=None ):
    """
    Display the specified axes of the arm at the specified set of angles
    """
    if ax is None:
        ax = gcf().add_subplot(111,projection='3d')
    mx = 0
    for a,g in zip(A, self.geom):
      ng = dot(a,g)
      mx = max(mx,abs(ng.max()))
      ax.plot3D( ng[0,:], ng[1,:], ng[2,:], '.-' )
    tp = dot(a, self.tool)
    ax.plot3D( [tp[0]], [tp[1]], [tp[2]], 'hk' )
    ax.plot3D( [tp[0]], [tp[1]], [tp[2]], '.y' )
    ax.plot3D( [-mx,mx], [-mx,mx], [-mx,mx], '.w', alpha=0.1 )
    
  def plotAll( self, ang ):
    """
    Plot arm in 3 views
    """
    A = self.at(ang)
    clf()
    ax = [-20,20,-20,20]
    subplot(2,2,1)
    self.plotIJ(A,0,1)
    axis('equal')
    axis(ax)
    grid(1)
    xlabel('X'); ylabel('Y')
    subplot(2,2,2)
    self.plotIJ(A,2,1)
    axis('equal')
    axis(ax)
    grid(1)
    xlabel('Z'); ylabel('Y')
    subplot(2,2,3)
    self.plotIJ(A,0,2)
    axis('equal')
    axis(ax)
    grid(1)
    xlabel('X'); ylabel('Z')
    ax = gcf().add_subplot(224,projection='3d')
    self.plot3D(A,ax)

