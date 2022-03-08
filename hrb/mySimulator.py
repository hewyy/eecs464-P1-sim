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
from pylab import randn,dot,mean,exp,newaxis,sign



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
    self.dur = 3
    # Number of intermediate steps
    self.N = 10
    # left(-1) right(+1) both(0)
    self.wheel = 0

  def behavior(self):
    s = self.simIX
    step = self.dist / float(self.N)
    dt = self.dur / float(self.N)
    for k in range(self.N):
      s.basewheels(step,self.wheel)
      yield self.forDuration(dt)


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
    self.dur = 3
    # Number of intermediate steps
    self.N = 10

  def behavior(self):
    s = self.simIX
    step = self.dist / float(self.N)
    dt = self.dur / float(self.N)
    for k in range(self.N):
      s.pulley(step)
      yield self.forDuration(dt)



class AutoMode( Plan ):
    def __init__(self,app,simIX,sensor,*arg,**kw):
        Plan.__init__(self,app,*arg,**kw)
        self.simIX = simIX
        self.sensor = sensor
        
        # Left(-1) Right(+1) Both(0)-default
        self.wheel = 0
        # Distance to travel
        self.xstep = 1
        self.ystep = 1
        # Number of intermediate steps
        self.N = 10

        self.waypoints = 3
        self.target = [0, 0]
        self.pos = [0, 0]
        
        # record each step
        self.goodstep = []
        self.status = 1
        self.done = 0
        
        # sensor
        self.f = 0
        self.b = 0
        
    
    def behavior(self):
        s = self.simIX
           
        while self.waypoints:
            # initialize for each waypoint 
            self.waypoints = len(self.sensor.lastWaypoints[1])-1
            self.target = self.sensor.lastWaypoints[1][1]
            self.pos = self.sensor.lastWaypoints[1][0]
            self.goodstep = []
            
            # calculate the dist and step
            self.xstep = (self.target[0]-self.pos[0]) / float(self.N)
            self.ystep = (self.target[1]-self.pos[1]) / float(self.N)
            
            # count the step
            step = 1
            self.done = 0
            
            while self.target == self.sensor.lastWaypoints[1][1]:
                progress("Step %d" % step)
                
                if not self.done:
                    # conduct one step
                    s.basewheels(-self.xstep,self.wheel) # x * cos(pi) = -x
                    yield self.forDuration(0.5)
                    s.pulley(self.ystep)
                    yield self.forDuration(2)
                
                # check and record the status of each step
                self.onLine()
                if self.status:
                    self.goodstep += [step]
                    
                    if step > 5 and min(self.f, self.b) < 10:
                        progress("Almost there! Only need to move the tag")
                        s.pulley(-self.ystep/2)
                        yield self.forDuration(2)
                        if len(self.sensor.lastWaypoints[1])-1 == self.waypoints - 1:
                            progress("Waypoint reached")
                            self.done = 1
                        if not self.done:
                            s.pulley(self.ystep)
                            yield self.forDuration(2)
                            if len(self.sensor.lastWaypoints[1])-1 == self.waypoints - 1:
                                progress("Waypoint reached")
                                self.done = 1
                        if not self.done:
                            s.pulley(-1.5*self.ystep)
                            yield self.forDuration(2)
                            if len(self.sensor.lastWaypoints[1])-1 == self.waypoints - 1:
                                progress("Waypoint reached")
                                self.done = 1
                        if not self.done:
                            s.pulley(2*self.ystep)
                            yield self.forDuration(2)
                            if len(self.sensor.lastWaypoints[1])-1 == self.waypoints - 1:
                                progress("Waypoint reached")
                                self.done = 1
                        if not self.done:
                            progress("Tried, but didn't reach.")
                    
                else: # robot is moving away from the line
                    progress("Correcting")
                    prev_f = self.f
                    prev_b = self.b
                    
                    # undo the last steps
                    progress("Undo the last step 1")
                    s.pulley(-self.ystep)
                    yield self.forDuration(1.5)
                    ts,f1,b1 = self.sensor.lastSensor
                    
                    progress("Undo the last step 2")
                    s.basewheels(self.xstep,self.wheel)
                    yield self.forDuration(1.5)
                    ts,f2,b2 = self.sensor.lastSensor
                    
                    if f1+b1 > f2+b2:
                        dirc = -1
                        s.pulley(dirc * 1.5*self.ystep)
                    else:
                        dirc = 1
                        s.pulley(dirc * 0.5*self.ystep)
                    yield self.forDuration(1.5)
                    self.onLine()
                    
                    if self.status:
                        progress("Correct direction! Back to the line.")
                    elif self.f > prev_f and self.b > prev_b:
                        progress("Not enough. Do more correction.")
                        s.pulley(dirc * self.ystep/3)
                        yield self.forDuration(1.5)
                        if self.onLine():
                            progress("Back to the line")
                        else:
                            progress("Status: %d. Not sure where to go." % self.status)
                    elif self.f < prev_f and self.b < prev_b:
                        progress("Get worse...")
                    else:
                        progress("Status: %d. Not sure where to go." % self.status)


                progress("Steps on line: " + str(self.goodstep))
                step += 1
                
                if step > self.N: # it should reach the waypoint now
                    yield self.forDuration(1)
                    if len(self.sensor.lastWaypoints[1])-1 == self.waypoints - 1:
                        progress("Waypoint reached")
                        self.done = 1
                    else:
                        while self.status and not self.done:
                            if self.f < 10 or self.b < 10:
                                progress("Almost there! Only need to move the tag")
                                s.pulley(self.ystep/2)
                                yield self.forDuration(2)
                                if len(self.sensor.lastWaypoints[1])-1 == self.waypoints - 1:
                                    progress("Waypoint reached")
                                    self.done = 1
                                else:
                                    s.pulley(-self.ystep)
                                    yield self.forDuration(2)
                                    if len(self.sensor.lastWaypoints[1])-1 == self.waypoints - 1:
                                        progress("Waypoint reached")
                                        self.done = 1
                            else:
                                progress("Extra Half Step Forward")
                                s.basewheels(-self.xstep/2,self.wheel)
                                yield self.forDuration(2)
                                if len(self.sensor.lastWaypoints[1])-1 == self.waypoints - 1:
                                    progress("Waypoint reached")
                                    self.done = 1
                                if not self.done:
                                    s.pulley(self.ystep/2)
                                    yield self.forDuration(2)
                                    if len(self.sensor.lastWaypoints[1])-1 == self.waypoints - 1:
                                        progress("Waypoint reached")
                                        self.done = 1
                            self.onLine()

                        if not self.status and not self.done: # off target
                            # undo part of the last step
                            s.basewheels(self.xstep/3,self.wheel)
                            yield self.forDuration(0.5)
                            s.pulley(-self.ystep/3)
                            yield self.forDuration(2)
                            if len(self.sensor.lastWaypoints[1])-1 == self.waypoints - 1:
                                progress("Waypoint reached")
                                self.done = 1
                            else:
                                progress("Off target! Move around. Hope to reach the waypoint.")
                                self.MoveAround()
            
            progress("*** One Waypoint reached ***")
            
            

    def onLine(self):
        ts,self.f,self.b = self.sensor.lastSensor
        if self.f + self.b < 230:
            progress("Off the line")
            self.status = 0
        else:
            progress("On the line")
            self.status = 1
        return self.status

    
    def MoveAround(self):
        s = self.simIX
        
        xstep = 2
        ystep = 2
        
        # left
        if not self.done:
            s.pulley(-ystep)
        yield self.forDuration(2)
        if len(self.sensor.lastWaypoints[1])-1 == self.waypoints - 1:
            self.done = 1
            return progress("Waypoint reached")
        # upper left
        if not self.done:
            s.basewheels(xstep, 0)
        yield self.forDuration(2)
        if len(self.sensor.lastWaypoints[1])-1 == self.waypoints - 1:
            self.done = 1
            return progress("Waypoint reached")
        # upper
        if not self.done:
            s.pulley(ystep)
        yield self.forDuration(2)
        if len(self.sensor.lastWaypoints[1])-1 == self.waypoints - 1:
            self.done = 1
            return progress("Waypoint reached")
        # upper right
        if not self.done:
            s.pulley(ystep)
        yield self.forDuration(2)
        if len(self.sensor.lastWaypoints[1])-1 == self.waypoints - 1:
            self.done = 1
            return progress("Waypoint reached")
        # right
        if not self.done:
            s.basewheels(-xstep, 0)
        yield self.forDuration(2)
        if len(self.sensor.lastWaypoints[1])-1 == self.waypoints - 1:
            self.done = 1
            return progress("Waypoint reached")
        # lower right
        if not self.done:
            s.basewheels(-xstep, 0)
        yield self.forDuration(2)
        if len(self.sensor.lastWaypoints[1])-1 == self.waypoints - 1:
            self.done = 1
            return progress("Waypoint reached")
        # lower
        if not self.done:
            s.pulley(-ystep)
        yield self.forDuration(2)
        if len(self.sensor.lastWaypoints[1])-1 == self.waypoints - 1:
            self.done = 1
            return progress("Waypoint reached")
        # lower left
        if not self.done:
            s.pulley(-ystep)
        yield self.forDuration(2)
        if len(self.sensor.lastWaypoints[1])-1 == self.waypoints - 1:
            self.done = 1
            return progress("Waypoint reached")
        
        return progress("One round done. Unreached.")



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
    # for manual mode
    self.basewheels = BaseWheels(self,self.robSim) 
    self.movingtag = Pulley(self,self.robSim)
    # for auto mode
    self.auto = AutoMode(self,self.robSim,self.sensor)
    

  def showSensors( self ):
    """
    Display sensor readings
    """
    # This code should help you understand how you access sensor information
    ts,f,b = self.sensor.lastSensor
    if ts:
      progress( "Sensor: %4d f %d b %d d %d" % (ts-self.T0,f,b,f-b)  )
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
        self.basewheels.dur = 4
        self.basewheels.N = 10
        self.basewheels.dist = 10.0
        self.basewheels.start()
        return progress("(say) Both wheels move forward together")
      elif evt.key == K_DOWN:
        self.basewheels.wheel = 0
        self.basewheels.dur = 4
        self.basewheels.N = 10
        self.basewheels.dist = -10.0
        self.basewheels.start()
        return progress("(say) Both wheels move backward together")
      if evt.key == K_a:
        self.basewheels.wheel = -1
        self.basewheels.dur = 3
        self.basewheels.N = 6
        self.basewheels.dist = 10.0
        self.basewheels.start()
        return progress("(say) Left wheel move forward")
      elif evt.key == K_z:
        self.basewheels.wheel = -1
        self.basewheels.dur = 3
        self.basewheels.N = 6
        self.basewheels.dist = -10.0
        self.basewheels.start()
        return progress("(say) Left wheel move backward")
      if evt.key == K_s:
        self.basewheels.wheel = 1
        self.basewheels.dur = 3
        self.basewheels.N = 6
        self.basewheels.dist = 10.0
        self.basewheels.start()
        return progress("(say) Right wheel move forward")
      elif evt.key == K_x:
        self.basewheels.wheel = 1
        self.basewheels.dur = 3
        self.basewheels.N = 6
        self.basewheels.dist = -10.0
        self.basewheels.start()
        return progress("(say) Right wheel move backward")
      if evt.key == K_LEFT:
        self.movingtag.dur = 1
        self.movingtag.N = 3
        self.movingtag.dist = -5.0
        self.movingtag.start()
        return progress("(say) Tag moves left")
      elif evt.key == K_RIGHT:
        self.movingtag.dur = 1
        self.movingtag.N = 3
        self.movingtag.dist = 5.0
        self.movingtag.start()
        return progress("(say) Tag moves right")
      if evt.key == K_g:
        progress("(say) Autonomous Mode On. Start!")
        self.auto.start()
    ### DO NOT MODIFY -----------------------------------------------
      else:# Use superclass to show any other events
        return JoyApp.onEvent(self,evt)
    return # ignoring non-KEYDOWN events

if __name__=="__main__":
  from sys import argv
  print("""
  Running the robot simulator

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
