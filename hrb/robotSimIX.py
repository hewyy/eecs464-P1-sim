# -*- coding: utf-8 -*-
"""
Created on Thu Sep  4 20:31:13 2014

@author: shrevzen-home
"""
from gzip import open as opengz
from json import dumps as json_dumps
from numpy import asfarray, dot, c_, mean, exp, sqrt, any, isnan, asarray
from numpy.linalg import svd
from numpy.random import randn
import math

DEFAULT_MSG_TEMPLATE = {
    0 : [[2016, 1070], [1993, 1091], [2022, 1115], [2044, 1093]],
    1 : [[1822, 1323], [1824, 1287], [1787, 1281], [1784, 1315]],
    2 : [[1795, 911], [1766, 894], [1749, 916], [1779, 933]],
    3 : [[1451, 876], [1428, 896], [1454, 917], [1476, 896]],
    #4 : [[1374, 1278], [1410, 1268], [1399, 1236], [1364, 1243]],
    4 : [[1409, 1278], [1409, 1243], [1374, 1243], [1374, 1278]],
    #4  : [[2276, 1179], [2311, 1171], [2309, 1206], [2273, 1204]],
    22 : [[1744, 622], [1743, 646], [1774, 650], [1774, 626]],
    23 : [[2274, 1171], [2312, 1177], [2306, 1146], [2271, 1141]],
    24 : [[1100, 975], [1110, 946], [1077, 938], [1066, 966]],
    25 : [[1666, 1629], [1665, 1589], [1625, 1585], [1626, 1624]],
    26 : [[2305, 1663], [2310, 1704], [2352, 1708], [2345, 1667]],
    27 : [[2230, 697], [2230, 721], [2262, 727], [2260, 704]],
    28 : [[911, 1525], [952, 1523], [953, 1483], [913, 1486]],
    29 : [[1222, 542], [1193, 537], [1186, 558], [1216, 566]],
}
try:
  from randArenaOutput import MSG_TEMPLATE, randSeed
  print("### Using randomly generated arena from seed %d" % randSeed)
except:
  MSG_TEMPLATE = DEFAULT_MSG_TEMPLATE
  print("### Using DEFAULT arena")

# NOTE: must be AFTER randAreaOutput import
from waypointShared import waypoints, corners, ROBOT_TAGID

from pdb import set_trace as DEBUG


def tags2list( dic ):
    """
    Convert a dictionary of tags into part of a list for JSON serialization

    INPUT:
      dic -- dictionary mapping tag id to 4x2 corner location array

    OUTPUT:
      list to be concatenated into JSON message
    """
    return [
        {
          'i' : k,
          'p': [ list(row) for row in v ]
        }
        for k,v in dic.items()
    ]

def findXing(a,b):
  """
  Find the crossing point of two lines, represented each by a pair of
  points on the line

  INPUT:
    a -- 2x2 -- two points, in rows
    b -- 2x2 -- two points, in rows

  OUTPUT: c -- 2 -- a point, as an array
  """
  a = asfarray(a)
  b = asfarray(b)
  # The nullspace of this matrix is the projective representation
  # of the intersection of the lines. Each column's nullspace is
  # one of the lines
  X = c_[a[1]-a[0],b[0]-b[1],a[0]-b[0]].T
  if X.ndim is not 2:
    DEBUG()
  Q = svd(X)[0]
  # Last singular vector is basis for nullspace; convert back from
  # projective to Cartesian representation
  q = Q[:2,2]/Q[2,2]
  c = q[0]*(a[1]-a[0])+a[0]
  return c

class RobotSimInterface( object ):
  """
  Abstract superclass RobotSimInterface defines the output-facing interface
  of a robot simulation.

  Subclasses of this class must implement all of the methods
  """
  def __init__(self, fn=None):
    """
    INPUT:
      fn -- filename / None -- laser log name to use for logging simulated
          laser data. None logged if name is None

    ATTRIBUTES:
      tagPos -- 4x2 float array -- corners of robot tag
      laserAxis -- 2x2 float array -- two points along axis of laser
      waypoints -- dict -- maps waypoint tag numbers to 4x2 float
          arrays of the tag corners
    """
    # Initialize dummy values into robot and arena state
    self.tagPos = asfarray(MSG_TEMPLATE[ ROBOT_TAGID[0]])
    self.laserAxis = dot([[1,1,0,0],[0,0,1,1]],self.tagPos)/2
    self.waypoints = { tid : asfarray(MSG_TEMPLATE[tid]) for tid in waypoints }
    ### Initialize internal variables
    # Two points on the laser screen
    self.laserScreen = asfarray([[-1,-1],[1,-1]])
    # Buffer for visualization requests
    self.visArenaClear()
    self.visRobotClear()
    # Cache for simulated TagStreamer messages
    self._msg = None
    # Output for simulated laser data
    if not fn:
      self.out = None
    else:
      self.out = opengz(fn,"w")

  def visArenaClear(self):
      """
      Reset / clear the arena visualization requests
      """
      self._lw = []

  def visRobotClear(self):
      """
      Reset / clear the robot visualization requests
      """
      self._lr = []

  def visArena(self,meth,*arg,**kw):
      """
      Add a visualization request to be plotted in the arena subplot
      INPUT:
        meth - str - axis method name
        *arg,**kw - additional arguments as appropriate for the visualilzation method

      NOTE: coordinates should be given in arena coordinates,
        i.e. with respect to the true world frame used to give
        the reference locations of the markers

      Example:
      >>> ix.visArenaClear()
      >>> ix.visArena('plot',x=[10,20,20,10,10],y=[10,10,20,20,10],c='r') # plot a red square
      """
      msg = { '@w' : meth }
      for n,v in enumerate(arg):
          msg['@%d' % n] = v
      msg.update(kw)
      self._lw.append(msg)

  def visRobot(self,meth,*arg,**kw):
      """
      Add a visualization request to be plotted in the robot subplot
      INPUT:
        meth - str - axis method name
        *arg,**kw - additional arguments as appropriate for the visualilzation method

      NOTE: coordinates should be given in arena coordinates,
        i.e. with respect to the true world frame used to give
        the reference locations of the markers
      """
      msg = { '@r' : meth }
      for n,v in enumerate(arg):
          msg['@%d' % n] = v
      msg.update(kw)
      self._lr.append(msg)

  def refreshState( self ):
    """<<pure>> refresh the value of self.tagPos and self.laserAxis"""
    print("<<< MUST IMPLEMENT THIS METHOD >>>")

  def getTagMsg( self ):
    """
    Using the current state, generate a TagStreamer message simulating
    the robot state
    """
    # Cache a list of the corner tags. They don't move
    if self._msg is None:
      self._msg = tags2list({ tid : asfarray(MSG_TEMPLATE[tid]) for tid in corners})
    # Collect robot and waypoint locations
    state = { ROBOT_TAGID[0] : self.tagPos }
    state.update( self.waypoints )
    # Combine all into a list
    msg = tags2list(state) + self._msg + self._lw + self._lr
    # Serialize
    return json_dumps(msg)

  def logLaserValue( self, now ):
    """
    Using the current state, generate a fictitious laser pointer reading
    INPUT:
      now -- float -- timestamp to include in the message

    OUTPUT: string of human readable message (not what is in log)
    """
    x = findXing( self.laserScreen, self.laserAxis )
    if any(isnan(x)):
        return "Laser: <<DISQUALIFIED>>"
    if self.out:
      self.out.write("%.2f, 1, %d, %d\n" % (now,x[0],x[1]))
    return "Laser: %d,%d " % tuple(x)

class SimpleRobotSim( RobotSimInterface ):
    def __init__(self, *args, **kw):
        RobotSimInterface.__init__(self, *args, **kw)
        self.dNoise = 0.1 # Distance noise
        self.aNoise = 0.02 # Angle noise
        self.lNoise = 0.01 # Laser pointing noise
        
        # make the tag a square
        fix_pos = self.squareUp(self.tagPos)

        # Model the tag
        tag = dot(fix_pos,[1,1j])
        self.zTag = tag-mean(tag)
        self.pos = mean(tag)
        self.ang = 1+0j
        self.tang = 0+1j

        
    def squareUp(self, tag):
        # resizes the tag to be square
        tag_height = 35
        tag_width = 35

        left_corner_x = tag[0][0]
        left_corner_y = tag[0][1]

        tag[0][0] = left_corner_x
        tag[0][1] = left_corner_y 


        tag[1][0] = left_corner_x + tag_width
        tag[1][1] = left_corner_y

        tag[2][0] = left_corner_x + tag_width
        tag[2][1] = left_corner_y + tag_height


        tag[3][0] = left_corner_x
        tag[3][1] = left_corner_y + tag_height


        return tag

    def move(self,dist):
        # Move in direction of self.ang
        #  If we assume Gaussian errors in velocity, distance error will grow
        #  as sqrt of goal distance
        self.pos += dist*self.ang #+ randn()*self.dNoise*sqrt(abs(dist))


    def turn(self,ang):
        # Turn by ang (plus noise)
        self.pos += ang*self.tang

    def refreshState(self):
        # Compute tag points relative to tag center, with 1st point on real axis
        tag = self.zTag  + self.pos #  *self.ang
        # New tag position is set by position and angle
        self.tagPos = c_[tag.real,tag.imag]
        

        print(self.tagPos)

        #Laser axis is based on tag and turret
        c = mean(tag) # center
        r = mean(abs(tag-c))
        ax = self.ang

        self.laserAxis = [
            [c.real,c.imag],[(c+ax).real, (c+ax).imag]
        ]


        vl = c + asarray([0,ax*100*r])
        # Start a new visual in robot subplot
        self.visRobotClear()
        # plot command in robot subplot,
        #   '~' prefix changes coordinates using homography
        self.visRobot('~plot',
            [int(v) for v in vl.real],
            [int(v) for v in vl.imag],
            c='g')

        # Start a new visual in arena subplot
        self.visArenaClear()
        # plot command in arena subplot,
        #   '~' prefix changes coordinates using homography
        self.visArena('~plot',
            [int(v) for v in vl.real],
            [int(v) for v in vl.imag],
            c='g',alpha=0.5)
        # We can call any plot axis class methods, e.g. grid
        #self.visArena('grid',1)

