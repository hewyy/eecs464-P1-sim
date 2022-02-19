# -*- coding: utf-8 -*-
"""
FILE: sensorPlan.py

Contains the SensorPlan class, which interfaces with the waypointServer to
give sensor readings from the robot

Created on Sat Sep  6 12:02:16 2014

@author: shrevzen-home
"""

from errno import EAGAIN
# Uses UDP sockets to communicate
from socket import (
  socket, AF_INET,SOCK_STREAM, IPPROTO_TCP, error as SocketError,
  )
# Packets are JSON enocoded
from json import loads as json_loads

# The main program is a JoyApp
from joy.plans import Plan, progress

# Include all the modeling functions provided to the teams
#  this ensures that what the server does is consistent with the model given
#  to students during the development process
from waypointShared import *

class SensorPlanTCP( Plan ):
  """
  SensorPlan is a concrete Plan subclass that uses a TCP socket to
  and decode WayPoint messages
  """
  def __init__( self, app, *arg, **kw ):
    if "port" in kw:
        sPort = int(kw.pop('port'))
    else:
        sPort = WAYPOINT_MSG_PORT
    if "server" in kw:
      self.svrAddr = (kw['server'],sPort)
      del kw['server']
    else:
      self.svrAddr = (WAYPOINT_HOST,sPort)
    Plan.__init__(self, app, *arg, **kw )
    self.sock = None
    self.lastSensor = (0,None,None)
    self.lastWaypoints = (0,[])
    self.buf = b''

  def _connect( self ):
    """Set up the socket"""
    s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)
    s.connect(self.svrAddr)
    s.setblocking(0)
    self.sock = s
    self.buf = b''
    progress("Sensor connected to %s:%d" % self.svrAddr)

  def stop( self ):
    """(called when stopping) clean up the socket, if there is one"""
    if self.sock is not None:
      self.sock.close()
    self.sock = None

  def ensureConnection( self ):
    """(sub-behavior) loops until the socket is up"""
    while True:
      # if not connected --> try to connect
      if self.sock is None:
        self._connect()
      # if not connected --> sleep for a bit
      if self.sock is None:
        yield self.forDuration(0.1)
      else: # otherwise --> done, return to parent
        return

  def sendto(self,*argv,**kw):
    """
    Expose the socket sendto to allow owner to use socket for sending

    Will try to connect a socket if there is none
    """
    # if not connected --> try to connect
    if self.sock is None:
      self._connect()
    return self.sock.sendto(*argv,**kw)

  def _nextMessage( self ):
        """
        Obtain the next message; kill socket on error.

        returns '' if nothing was received
        """
        # if buffer contains no complete messages --> read socket
        if self.buf.find(b'}')<0:
            # receive an update / skip
            try:
              msg = self.sock.recv(1024)
            except SocketError as se:
              # If there was no data on the socket
              #   --> not a real error, else kill socket and start a new one
              if se.errno != EAGAIN:
                progress("Connection failed: "+str(se))
                self.sock.close()
                self.sock = None
              return b''
            self.buf = self.buf + msg
        # Use previously buffered data
        buf = self.buf
        # End of dictionary should be end of message
        f = buf.find(b"}")
        if f<0:
          return b''
        # Pull out the first dictionary
        self.buf = buf[f+1:]
        return buf[:f+1]

  def behavior( self ):
    """
    Plan main loop
    """
    while True:
      # If no socket set up --> activate the ensureConnection sub-behavior to fix
      if self.sock is None:
        yield self.ensureConnection()
      msg = self._nextMessage()
      # If no message --> sleep a little and try again
      if not msg:
          yield self.forDuration(0.3)
          continue
      # Parse the message
      dic = json_loads(msg)
      ts = self.app.now
      self.lastSensor = (ts, dic['f'], dic['b'])
      if "w" in dic:
        self.lastWaypoints = (ts,dic['w'])
      # NOTE: does not yield if flooded with traffic


if __name__=="__main__":
  import sys
  from joy import JoyApp
  from joy.decl import *

  print("""
  Running the sensor reader

  Connects a SensorPlanTCP to a waypointTask do display the sensor readings

  Useage: %s <server-IP>
  """ % sys.argv[0])

  class SensorApp( JoyApp ):
    """Concrete class SensorApp <<singleton>>
    """
    def __init__(self,wphAddr=WAYPOINT_HOST,*arg,**kw):
      JoyApp.__init__( self,
        confPath="$/cfg/JoyApp.yml", *arg, **kw
        )
      self.srvAddr = (wphAddr, APRIL_DATA_PORT)

    def onStart( self ):
      # Set up the sensor receiver plan
      self.sensor = SensorPlanTCP(self,server=self.srvAddr[0])
      self.sensor.start()
      progress("Using %s:%d as the waypoint host" % self.srvAddr)
      self.timeForStatus = self.onceEvery(0.33)
      self.T0 = self.now

    def showSensors( self ):
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


    def onEvent( self, evt ):
      # periodically, show the sensor reading we got from the waypointServer
      if self.timeForStatus():
        self.showSensors()
      return JoyApp.onEvent(self,evt)

  if len(sys.argv)>1:
      app=SensorApp(wphAddr=sys.argv[1], cfg={'windowSize' : [160,120]})
  else:
      app=SensorApp(wphAddr=WAYPOINT_HOST, cfg={'windowSize' : [160,120]})
  app.run()
