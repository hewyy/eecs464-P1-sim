# -*- coding: utf-8 -*-
"""
Created on Thu Sep  4 20:31:13 2014

@author: shrevzen-home
"""
from gzip import open as opengz
from json import dumps as json_dumps
from numpy import asfarray, dot, c_, mean, exp, sqrt, any, isnan, asarray, array, zeros, linspace, full
from numpy.linalg import svd, inv
from numpy.random import randn
from skimage import transform
import math


from waypointShared import (ref, fitHomography, corners, DEFAULT_ref)


DEFAULT_MSG_TEMPLATE = {
    0 : [[2016, 1070], [1993, 1091], [2022, 1115], [2044, 1093]],
    1 : [[1822, 1323], [1824, 1287], [1787, 1281], [1784, 1315]],
    2 : [[1795, 911], [1766, 894], [1749, 916], [1779, 933]],
    3 : [[1451, 876], [1428, 896], [1454, 917], [1476, 896]],
    4 : [[1374, 1278], [1410, 1268], [1399, 1236], [1364, 1243]],
    22 : [[1744, 622], [1743, 646], [1774, 650], [1774, 626]],
    23 : [[2274, 1171], [2312, 1177], [2306, 1146], [2271, 1141]],
    24 : [[1100, 975], [1110, 946], [1077, 938], [1066, 966]],
    25 : [[1666, 1629], [1665, 1589], [1625, 1585], [1626, 1624]],
    26 : [[2305, 1663], [2310, 1704], [2352, 1708], [2345, 1667]],
    27 : [[2230, 697], [2230, 721], [2262, 727], [2260, 704]],
    28 : [[911, 1525], [952, 1523], [953, 1483], [913, 1486]],
    29 : [[1222, 542], [1193, 537], [1186, 558], [1216, 566]],
    666 : [[10, 10], [-10, 10], [-10, -10], [10, -10]] # hail satan
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
    self.tagPos = asfarray(MSG_TEMPLATE[4])
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

        # calculate the homography matrix
        self.homomatrix = self.calHomography()

        # make the tag a square
        self.dNoise = 0.1 # Distance noise
        self.aNoise = 0.02 # Angle noise
        self.lNoise = 0.01 # Laser pointing noise
        
        self.lineUp()
        self.laserAxis = dot([[1,1,0,0],[0,0,1,1]],self.tagPos)/2
               
        # tag with respect to arena
        self.tagPosArena = self.transformToArena(self.tagPos)
        tagArena = dot(self.tagPosArena, [1.0, 1.0j])
        self.zTagArena = tagArena-mean(tagArena)
        self.posArena = mean(tagArena)

        # tag with respect to camera
        tag = dot(self.tagPos,[1.0,1.0j])
        self.zTag = tag-mean(tag)
        self.pos = mean(tag)
        self.ang = 1+0j
        self.tang = 0+1j

    def lineUp(self):
        # transform the spawn position into arena coors
        robot_pos_arena = self.transformToArena(self.tagPos)

        robot_width = 5
        robot_pos_arena[0][0] = 100# round(robot_pos_arena[0][0])
        robot_pos_arena[0][1] = 50 #round(robot_pos_arena[0][1])
        
        robot_pos_arena[1][0] = 100 - robot_width#round(robot_pos_arena[1][0])
        robot_pos_arena[1][1] = 50 #robot_pos_arena[0][1]

        robot_pos_arena[2][0] = 100 - robot_width #robot_pos_arena[1][0]
        robot_pos_arena[2][1] = 50 + robot_width #round(robot_pos_arena[2][1])

        robot_pos_arena[3][0] = 100 #robot_pos_arena[0][0]
        robot_pos_arena[3][1] = 50 + robot_width #robot_pos_arena[2][1]

        # transform to the camera view
        self.tagPos = self.transformToCamera(robot_pos_arena)
        return

    def calHomography(self):
        """
        Used to calculate the homography matrix
        Returns the homography matrix

        Only called once
        """
        h = zeros((8,3),dtype=float)

        # TODO: make more 'pythony'

        # find corners of the arena in terms of the camera perspective
        for nm in range(len(corners)):

          corner = array(MSG_TEMPLATE[corners[nm]])

          h[nm][0] = mean(corner[:,0])/100
          h[nm][1] = mean(corner[:,1])/100

          # this is needed to make it a homogeneous coordinate
          # cartesian -> homogenous (just add a 1 )
          h[nm][2] = 1

        # Homography mapping roi to ref
        return fitHomography( h, DEFAULT_ref)

    def transformToArena(self, camera_coor):
      """
      Transfroms from camera coordinates to arena coordinates

      Returns points with arena coordinates
      """
      # Robot location with respect to the camera is in self.tagPos
      # take the average of the of the tags to get the middle of the robot
      starting_spot = zeros((4,3),dtype=float)

      # TODO: make more pythony
      for i in range(4):
        starting_spot[i][0] = camera_coor[i][0] / 100
        starting_spot[i][1] = camera_coor[i][1] / 100
        starting_spot[i][2] = 1

      trans = dot(starting_spot, self.homomatrix)

      # trans is now arena location of the robot
      arr = zeros((4,2),dtype=float)
      arr[:,0] = trans[:,0] / trans[:,2]
      arr[:,1] = trans[:,1] / trans[:,2]
        
      # find the middle point
      x = mean(arr[:,0])
      y = mean(arr[:,1])

      return arr

    def transformToCamera(self, arena_coor):
      """
      Transfroms from arena coordinates to camera coordinates

      Returns points with camera coordinates
      """
      # starting point
      starting_spot = zeros((4,3),dtype=float)

      # TODO: make more pythony
      for i in range(4):
        starting_spot[i][0] = arena_coor[i][0]
        starting_spot[i][1] = arena_coor[i][1]
        starting_spot[i][2] = 1

      trans = dot(starting_spot, inv(self.homomatrix))

      arr = zeros((4,2),dtype=float)
      arr[:,0] = trans[:,0] / trans[:,2]
      arr[:,1] = trans[:,1] / trans[:,2]

      arr = arr*100 # need the scaler

      return arr

    def move(self,dist):
        # Move in direction of self.ang
        #  If we assume Gaussian errors in velocity, distance error will grow
        #  as sqrt of goal distance
        self.posArena += dist*self.tang #+ randn()*self.dNoise*sqrt(abs(dist))

    def get_x(self):
      return self.posArena.real

    def get_y(self):
      return self.posArena.imag

    def turn(self,dist):
        # Turn by ang (plus noise)
        self.posArena += dist*self.ang

    def refreshState(self):
        # Compute tag points relative to tag center
        tagArena = self.zTagArena  + self.posArena #  *self.ang

        # New tag position is set by position and angle
        self.tagPosArena = c_[tagArena.real,tagArena.imag]
        self.tagPos = self.transformToCamera(self.tagPosArena)

        # TODO: draw the laser

        return