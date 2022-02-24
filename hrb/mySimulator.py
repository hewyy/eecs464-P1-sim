#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Feb 21 12:57:08 2022

@author: molanz
"""

from sensorPlanTCP import SensorPlanTCP
from mySimIX import SimpleRobotSim,RobotSimInterface
from joy import JoyApp, progress
from joy.decl import *
from joy.plans import Plan
from waypointShared import (
    WAYPOINT_HOST, WAYPOINT_MSG_PORT, APRIL_DATA_PORT
    )
from socket import (
  socket, AF_INET,SOCK_DGRAM, IPPROTO_UDP, error as SocketError,
  )
from pylab import randn,dot,mean,exp,newaxis, sign



class BaseWheels( Plan ):
  """
  The base have two wheels to move forward or back. -- y axis

  """
  def __init__(self,app,simIX):
    Plan.__init__(self,app)
    self.simIX = simIX
    # Distance to travel
    self.dist = 10
    # Duration of travel [sec]
    self.dur = 30
    # Number of intermediate steps
    self.N = 30
    # left(-1) right(+1) both(0)
    self.wheel = 0

  def behavior(self):
    s = self.simIX
    step = self.dist / float(self.N)
    dt = self.dur / float(self.N)
    for k in range(self.N):
      ori = s.basewheels(step,self.wheel)
      yield self.forDuration(dt)
    return ori


class Pulley( Plan ):
  """
  Use the pulley to move the tag along the rope. -- x axis

  """
  def __init__(self,app,simIX):
    Plan.__init__(self,app)
    self.simIX = simIX
    # Distance to travel
    self.dist = 10
    # Duration of travel [sec]
    self.dur = 20
    # Number of intermediate steps
    self.N = 20

  def behavior(self):
    s = self.simIX
    step = self.dist / float(self.N)
    dt = self.dur / float(self.N)
    for k in range(self.N):
      ori = s.pulley(step)
      yield self.forDuration(dt)
    return ori

# class AutoMode( Plan ):
#     def __init__(self,app,task,sensor,*arg,**kw):
#         Plan.__init__(self,app,*arg,**kw)
#         self.task = task
#         self.sensor = sensor
    
#     def behavior(self):
#         # WP_record = self.sensor.lastWaypoints[1][1:]
#         times = len(self.sensor.lastWaypoints[1])-1
        
#         while times:
#             self.task.start()
#             yield self.forDuration(5)
#             times -= 1
            

class ReachOne( Plan ):
    def __init__(self,app,base,tag,sensor,*arg,**kw):
        Plan.__init__(self,app,*arg,**kw)
        self.xmove = base
        self.ymove = tag
        self.sensor = sensor
        self.target = [0, 0]
        self.pos = [0, 0]
    
    def behavior(self):
        self.target = self.sensor.lastWaypoints[1][1]
        startpoint = self.sensor.lastWaypoints[1][0]
        self.pos = startpoint
        self.angle = 0
        waypoint = len(self.sensor.lastWaypoints[1])
        
        while not len(self.sensor.lastWaypoints[1]) == waypoint-1 :
            while self.target == self.sensor.lastWaypoints[1][1]:
                
                if abs(self.target[1]-self.pos[1]) > abs(self.target[0]-self.pos[0]):
                    y_dist = self.target[1]-self.pos[1]
                    self.ymove.dist = y_dist
                    ori = self.ymove.start()
                    self.pos = ori[0]
                    yield self.forDuration(10)
                
                x_dist = self.target[0]-self.pos[0]
                self.xmove.wheel = 0
                self.xmove.dist = -x_dist
                ori = self.xmove.start()
                self.angle = ori[1]
                yield self.forDuration(10)
                
                times = 3
                while abs(self.angle) > 0.05 and times > 0:
                    if self.angle > 0.05:
                        self.xmove.wheel = -sign(x_dist)
                        self.xmove.dist = sign(x_dist)*100*sin(self.angle)
                        ori = self.xmove.start()
                        self.angle = ori[1]
                    elif self.angle < -0.05:
                        self.xmove.wheel = sign(x_dist)
                        self.xmove.dist = -sign(x_dist)*100*sin(self.angle)
                        ori = self.xmove.start()
                        self.angle = ori[1]
                    times -= 1
                    yield self.forDuration(10)
                
                self.pos = ori[0]             
                y_dist = self.target[1]-self.pos[1]
                self.ymove.dist = y_dist
                ori = self.ymove.start()
                self.pos = ori[0]
                yield self.forDuration(10)
                self.target = self.sensor.lastWaypoints[1][1]

            yield self.forDuration(50)
            if not len(self.sensor.lastWaypoints[1]) > 1:
                yield self.forDuration(50)
                
        # if not len(self.sensor.lastWaypoints[1]) == waypoint-1 :
        #     while self.target == self.sensor.lastWaypoints[1][1]:
        #         x_dist = self.target[0]-self.pos[0]
        #         y_dist = self.target[1]-self.pos[1]
        #         self.xmove.wheel = 0
        #         self.xmove.dist = -x_dist
        #         self.xmove.start()
        #         # self.pos = self.xmove.start()
        #         yield self.forDuration(10)
        #         self.ymove.dist = y_dist
        #         self.pos = self.ymove.start()
        #         yield self.forDuration(10)
        #         self.target = self.sensor.lastWaypoints[1][1]
        #     yield self.forDuration(10)
        # yield self.forDuration(10)


        
            

class RobotSimulatorApp( JoyApp ):
  """Concrete class RobotSimulatorApp <<singleton>>
     A JoyApp which runs the DummyRobotSim robot model in simulation, and
     emits regular simulated tagStreamer message to the desired waypoint host.

     Used in conjection with waypointServer.py to provide a complete simulation
     environment for Project 1
  """
  def __init__(self,wphAddr=WAYPOINT_HOST,wphPort=WAYPOINT_MSG_PORT,*arg,**kw):
    """
    Initialize the simulator
    """
    JoyApp.__init__( self,
      confPath="$/cfg/JoyApp.yml", *arg, **kw
      )
    self.srvAddr = (wphAddr, wphPort)
    # ADD pre-startup initialization here, if you need it

  def onStart( self ):
    """
    Sets up the JoyApp and configures the simulation
    """
    ### DO NOT MODIFY ------------------------------------------
    # Set up socket for emitting fake tag messages
    s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)
    s.bind(("",0))
    self.sock = s
    # Set up the sensor receiver plan
    self.sensor = SensorPlanTCP(self,server=self.srvAddr[0],port=self.srvAddr[1])
    self.sensor.start()
    self.timeForStatus = self.onceEvery(1)
    self.timeForLaser = self.onceEvery(1/15.0)
    self.timeForFrame = self.onceEvery(1/20.0)
    progress("Using %s:%d as the waypoint host" % self.srvAddr)
    self.T0 = self.now
    ### MODIFY FROM HERE ------------------------------------------
    self.robSim = SimpleRobotSim(fn=None)
    self.basewheels = BaseWheels(self,self.robSim) 
    self.movingtag = Pulley(self,self.robSim)
    self.task = ReachOne(self,self.basewheels,self.movingtag,self.sensor)
    # self.auto = AutoMode(self,self.task,self.sensor)
    

  def showSensors( self ):
    """
    Display sensor readings
    """
    # This code should help you understand how you access sensor information
    ts,f,b = self.sensor.lastSensor
    if ts:
      progress( "Sensor: %4d f %d b %d" % (ts-self.T0,f,b)  )
    else:
      progress( "Sensor: << no reading >>" )
    ts,w = self.sensor.lastWaypoints
    if ts:
      progress( "Waypoints: %4d " % (ts-self.T0) + str(w))
    else:
      progress( "Waypoints: << no reading >>" )

  def emitTagMessage( self ):
    """Generate and emit and update simulated tagStreamer message"""
    #### DO NOT MODIFY --- it WILL break the simulator
    self.robSim.refreshState()
    # Get the simulated tag message
    msg = self.robSim.getTagMsg()
    # Send message to waypointServer "as if" we were tagStreamer
    self.sock.sendto(msg.encode("ascii"), (self.srvAddr[0],APRIL_DATA_PORT))

  def onEvent( self, evt ):
    #### DO NOT MODIFY --------------------------------------------
    # periodically, show the sensor reading we got from the waypointServer
    if self.timeForStatus():
      self.showSensors()
      progress( self.robSim.logLaserValue(self.now) )
      # generate simulated laser readings
    elif self.timeForLaser():
      self.robSim.logLaserValue(self.now)
    # update the robot and simulate the tagStreamer
    if self.timeForFrame():
      self.emitTagMessage()
    #### MODIFY FROM HERE ON ----------------------------------------
    if evt.type == KEYDOWN:
      if evt.key == K_UP:
        self.basewheels.wheel = 0
        self.basewheels.dist = 10.0
        self.basewheels.start()
        return progress("(say) Both wheels move forward together")
      elif evt.key == K_DOWN:
        self.basewheels.wheel = 0
        self.basewheels.dist = -10.0
        self.basewheels.start()
        return progress("(say) Both wheels move backward together")
      if evt.key == K_a:
        self.basewheels.wheel = -1
        self.basewheels.dist = 10.0
        self.basewheels.start()
        return progress("(say) Left wheel move forward")
      elif evt.key == K_z:
        self.basewheels.wheel = -1
        self.basewheels.dist = -10.0
        self.basewheels.start()
        return progress("(say) Left wheel move backward")
      if evt.key == K_s:
        self.basewheels.wheel = 1
        self.basewheels.dist = 10.0
        self.basewheels.start()
        return progress("(say) Right wheel move forward")
      elif evt.key == K_x:
        self.basewheels.wheel = 1
        self.basewheels.dist = -10.0
        self.basewheels.start()
        return progress("(say) Right wheel move backward")
      if evt.key == K_LEFT:
        self.movingtag.dist = -10.0
        self.movingtag.start()
        return progress("(say) Tag moves left")
      elif evt.key == K_RIGHT:
        self.movingtag.dist = 10.0
        self.movingtag.start()
        return progress("(say) Tag moves right")
      if evt.key == K_1:
        progress("(say) Heading to the next waypoint")
        self.task.start()
        return progress("(say) Stop Moving")
      if evt.key == K_g:
        progress("(say) Autonomous Mode On. Start!")
        times = 3
        while times:
            progress("(say) Heading to the target")
            self.task.start()
            times -= 1
        if len(self.sensor.lastWaypoints[1]) == 2:
            progress("Second Waypoint reached")
        else:
            progress("Second Try")
        times = 3
        while times:
            progress("(say) Heading to the target")
            self.task.start()
            times -= 1
        if len(self.sensor.lastWaypoints[1]) == 2:
            progress("(say) Last Try")
            times = 3
            while times:
                progress("(say) Heading to the next waypoint")
                self.task.start()
                times -= 1
        return progress("(say) Stop Moving")
      
    ### DO NOT MODIFY -----------------------------------------------
      else:# Use superclass to show any other events
        return JoyApp.onEvent(self,evt)
    return # ignoring non-KEYDOWN events

if __name__=="__main__":
  from sys import argv
  print("""
  Running the robot simulatorx

  Listens on local port 0xBAA (2986) for incoming waypointServer
  information, and also transmits simulated tagStreamer messages to
  the waypointServer at ip and port given on commandline

  USAGE:
    %s
        Connect to default host and port
    %s <host>
        Connect to specified host on default port
    %s <host> <port>
        Connect to specified host on specified port
  """ % ((argv[0],)*3))
  import sys
  cfg = {'windowSize' : [160,120]}
  if len(argv)>2:
      app=RobotSimulatorApp(wphAddr=argv[1],wphPort=int(argv[2]),cfg=cfg)
  elif len(argv)==2:
      app=RobotSimulatorApp(wphAddr=argv[1],cfg=cfg)
  else:
      app=RobotSimulatorApp(cfg=cfg)
  app.run()
