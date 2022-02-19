# file waypointShared.py contains definitions that are shared between the simulation
#   code in simTagStreamer and the waypointServer

## April tags associations ##########################################

from numpy import (
    array, inf, asarray, uint8, empty, mean, newaxis, dot, c_,
    kron, concatenate, zeros_like
)
from numpy.random import randn
from numpy.linalg import svd

# Port for TagStreamer data
APRIL_DATA_PORT = 0xB00

# Host for waypointServer
#WAYPOINT_HOST = "141.213.30.33"
WAYPOINT_HOST = "10.0.0.1" # we are using the VPN

# Port for Waypoint messages
WAYPOINT_MSG_PORT = 8080


# Extremal corners of the arena
excorners = [26,27,28,29]

# Tag ID for robot
ROBOT_TAGID = [ 4 ]

# Tag IDs for waypoints
waypoints = [0,1,2,3]

# Boundary of the arena, in order
DEFAULT_corners = [26,23,27,22,29,24,28,25]
# Reference locations of corners, with 1 in the last coordinate
ref0 = array([
    [66, 66, 66.1, 31.9, 0, 0, 0, 32.5 ],
    [0, 49.1, 86, 86.15, 89.35, 50.75, 0, 0],
    [0]*8 ])
ref0 *= 2.56
ref0 = ref0 - mean(ref0,1)[:,newaxis]
ref0[2,:] = 1
DEFAULT_ref = dot([[0,-1,0],[-1,0,0],[0,0,1]],ref0).T

try:
  from randArenaOutput import corners, ref, randSeed
  print("### Using randomized ref with seed %d" % randSeed)
except ImportError:
  corners = DEFAULT_corners
  ref = DEFAULT_ref
  print("### Using DEFAULT ref")
  
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

def lineSensorResponse( d, noise ):
  """
  Convert distances from line to line sensor measurements

  INPUT:
    d -- float(s) -- distance(s) from line (inf is allowed)
    noise -- float(s) -- scale of noise for measurements

    The shapes of d and noise must be the same, or noise should be
    scalar.

  OUTPUT: res
    for scalar inputs, res is an int in the range 0..255
    for array inputs, res is a uint8 array shaped like d+noise
  """
  d = abs(asarray(d).real)
  noise = asarray(noise)
  if noise.shape and (noise.shape != d.shape):
    raise ValueError("d.shape=%s is different from noise.shape=%s" % (str(d.shape),str(noise.shape)))
  res0 = 1/(1+d**2) + randn(*d.shape) * noise
  res1 = asarray(res0.clip(0,0.9999)*256, uint8)
  if res1.shape:
    return res1
  return int(res1)

def lineDist( c, a, b, scale=1.0, withZ = False ):
  """
  Compute distance of point(s) c, to line segment from a to b

  INPUT:
    c -- complex array of any shape -- sensor locations
    a,b -- complex -- endpoints of line segment

  OUTPUT: res (withZ=False) or res,z (withZ=True)
    res -- same shape as c -- distances
    z -- normalized complex positions (*private*)
  """
  # Rigid transform and rescaling that
  # takes (a,b) to (0,1), applied to c
  c = asarray(c)
  z = (c-a)/(b-a)
  far = (z.real<0) | (z.real>1)
  res = empty(z.shape,float)
  res[far] = inf
  d = z[~far].imag * abs(b-a)
  res[~far] = d/scale
  if withZ:
      return res,z
  return res

class Sensor( object ):
  def __init__(self, *lineargs, **linekw):
    self.lineargs = lineargs
    self.linekw = linekw
    self.noise = 0.01

  def sense( self, ax, a, b, c, scale=0.2 ):
    """
    Compute sensor measurement for line from a to b, given
    sensor location(s) c and a scale factor

    INPUT:
       ax -- matplotlib axis to draw on (or None)
       a,b -- complex
       c -- array of complex

    OUTPUT: shape of c
    """
    c = asarray(c)
    d,z = lineDist(c,a,b,scale=scale,withZ=True)
    res = asarray(lineSensorResponse(d,self.noise),int)
    if ax is None:
        return res
    x = z.real * (b-a) + a
    ax.plot( c_[c.real, x.real].T, c_[c.imag, x.imag].T,
      *self.lineargs, **self.linekw )
    if c.ndim is 0:
        z = (c+x)/2
        ax.text(z.real,z.imag,"%d" % res, ha='center',va='center' )
    else:
        for z,v in zip((c+x)/2,res):
            ax.text(z.real,z.imag,"%d" % v, ha='center',va='center' )
    return res
