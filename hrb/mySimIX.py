#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 21 16:39:03 2022

@author: molanz
"""

from gzip import open as opengz
from json import dumps as json_dumps
from numpy import asfarray, dot, c_, mean, exp, sqrt, any, isnan, asarray
from numpy.linalg import svd, inv, norm
from numpy.random import randn
from numpy import array, size, ones, hstack
from numpy import pi, sin, cos, arcsin, arccos, sign

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
}
try:
  from randArenaOutput import MSG_TEMPLATE, randSeed
  print("### Using randomly generated arena from seed %d" % randSeed)
except:
  MSG_TEMPLATE = DEFAULT_MSG_TEMPLATE
  print("### Using DEFAULT arena")

# NOTE: must be AFTER randAreaOutput import
from waypointShared import (
    ref, fitHomography, waypoints, corners, ROBOT_TAGID
)

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
  if X.ndim != 2:
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
    self.base = ones(5)
    # self.tagPos = asfarray(MSG_TEMPLATE[ROBOT_TAGID[0]])
    self.tagPos = asfarray(MSG_TEMPLATE[waypoints[0]])
    self.laserAxis = asfarray([[0, 0],[0, -1]])
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
        
        # arena_coor = dot(camera_coor, P)
        self.P = self.calHomography()
        
        # FIXME
        self.dNoise = 1 # Distance noise 0.1
        self.aNoise = 0.02 # Angle noise 0.02
        self.lNoise = 0.05 # Laser pointing noise 0.01
        
        # Model the tag in arena coordinates
        # tag = dot(self.tagPos,[1,1j])
        tag_a = self.CameraToArena(self.tagPos)
        self.zTag = tag_a-mean(tag_a)
        self.pos = mean(tag_a)
        self.ang = 1+0j
        self.tang = 1+0j
        self.basewidth = 200*0.75
        self.base_a = mean(tag_a).real + 0j


    def getangle(self,ang): # angle from +x axis
        return arccos(ang.real / norm(ang)) * sign(ang.imag)
    
    def ext(self,coor):
        # extend the third "1" column (n, 2) -> (n, 3)
        return hstack((coor, ones((size(coor,0),1))))
    
    def calHomography(self):
        # Calculate the projection matrix from camera to arena coordinates
        # Collect the corner tags and estimate homography
        roi = array([ mean(MSG_TEMPLATE[nmi],0) for nmi in corners ])
        roi = self.ext(roi)
        # Homography mapping roi(camera) to ref(arena)
        return fitHomography( roi, ref )
        
    def CameraToArena(self, tagPos):
        """
        Transfrom points from camera coordinates to arena coordinates
        Returns points in arena coordinates
        * Input(tagPos): (4,2) array
        * Output(tag_a): (4,) array of complex
        """
        tag_a = dot(self.ext(tagPos), self.P)
        tag_a = tag_a[:,0]/tag_a[:,2] + 1j * tag_a[:,1]/tag_a[:,2]
        return tag_a
        
    def ArenaToCamera(self, tagPos_a):
        """
        Transfrom points from arena coordinates to camera coordinates
        Returns points in camera coordinates
        * Input(tagPos_a): (4,2) array
        * Output(tag): (4,) array of complex
        """
        tag = dot(self.ext(tagPos_a), inv(self.P))
        tag = tag[:,0]/tag[:,2] + 1j * tag[:,1]/tag[:,2]
        return tag

    
    def basewheels(self,dist,wheel):
        # Move one wheel in a arc; move both in y direction
        # wheel: -1 (left wheel), +1 (right wheel), 0 (both)
        
        if wheel == 0:
            d_dist = -dist + randn()*self.dNoise*sqrt(abs(dist))
            self.pos += self.ang * d_dist
            self.base_a += self.ang * d_dist
            self.ang *= exp(1j*(randn()*self.aNoise))
            self.tang *= exp(1j*(randn()*self.aNoise))
        else:
            theta = wheel * arcsin(dist / self.basewidth)
            self.ang *= exp(1j*(theta + randn()*self.aNoise))
            self.tang *= exp(1j*(theta + randn()*self.aNoise))
            delta_pos = self.basewidth * sin(theta/2) + randn()*self.dNoise*sqrt(abs(theta*self.basewidth/2))
            self.pos += -wheel * delta_pos * exp(1j*theta/2)
            self.base_a += -wheel * delta_pos * exp(1j*theta/2)


    def pulley(self,dist):
        # Move tag along the rope
        angle = self.getangle(self.ang) + pi/2
        temp = self.pos + exp(1j*angle) * (dist + randn()*self.dNoise*sqrt(abs(dist)))
        if norm(temp-self.base_a) > self.basewidth/2:
            flag = sign((temp-self.base_a).imag)
            self.pos = self.base_a + flag * exp(1j*angle) * self.basewidth/2
        else:
            self.pos = temp


    # def calibration(self, dirc, err):
    #     angle = self.getangle(self.ang)
    #     if angle > err:
    #         wheel = dirc
    #         dist = -dirc*self.basewidth/2*sin(angle)
    #     elif angle < -err:
    #         wheel = -dirc
    #         dist = dirc*self.basewidth/2*sin(angle)
    #     self.basewheels(dist, wheel)


    # def feedback(self):
    #     return c_[self.pos.real,self.pos.imag][0], self.getangle(self.ang)


    def basePos(self):
        L = 10
        W = self.basewidth/2
        angle = self.getangle(self.ang)
        axis = array([self.base_a + L*exp(1j*angle), 
                      self.base_a - L*exp(1j*angle)])
        
        basePos = asarray([axis[0]+W*(-sin(angle)+cos(angle)*1j), 
                           axis[0]+W*(sin(angle)-cos(angle)*1j),
                           axis[1]+W*(sin(angle)-cos(angle)*1j),
                           axis[1]+W*(-sin(angle)+cos(angle)*1j)])
        return c_[basePos.real,basePos.imag]
    

    def refreshState(self):
        # Compute tag points relative to tag center, with 1st point on real axis
        # New tag position is set by position and angle
        tag_a = self.zTag * self.ang + self.pos
        tagPos_a = c_[tag_a.real,tag_a.imag]
        basePos_a = self.basePos()
        angle = self.getangle(self.ang)
        axis_a = array([self.pos + 10*exp(1j*angle), 
                      self.pos - 10*exp(1j*angle)])
        axis_a = c_[axis_a.real,axis_a.imag]
        # Transform back to camera coordinates
        tag = self.ArenaToCamera(tagPos_a)
        self.tagPos = c_[tag.real,tag.imag]
        base = self.ArenaToCamera(basePos_a)
        self.base = hstack((base, base[0])) #(5,) closed rectangle
        laserAxis = self.ArenaToCamera(axis_a)
        self.laserAxis = c_[laserAxis.real,laserAxis.imag]
        c = mean(tag) # center
        # Visualize laser
        vl = array([c, (c-laserAxis[0])*100+laserAxis[0] + randn()*self.lNoise*1j])
        
        # Start a new visual in robot subplot
        self.visRobotClear()
        # plot command in robot subplot,
        #   '~' prefix changes coordinates using homography
        self.visRobot('~plot',
            [int(v) for v in vl.real],
            [int(v) for v in vl.imag],
            c='g')
        self.visRobot('~plot',
            [int(x) for x in self.base.real],
            [int(y) for y in self.base.imag],
            c='r',alpha=0.3)
        # Start a new visual in arena subplot
        self.visArenaClear()
        # plot command in arena subplot,
        #   '~' prefix changes coordinates using homography
        self.visArena('~plot',
            [int(v) for v in vl.real],
            [int(v) for v in vl.imag],
            c='g',alpha=0.5)
        self.visArena('~plot',
            [int(x) for x in self.base.real],
            [int(y) for y in self.base.imag],
            c='r',alpha=0.3)
        # We can call any plot axis class methods, e.g. grid
        self.visArena('grid',1)
        
