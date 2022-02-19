#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 14:23:10 2020

@author: shrevzen
"""

from numpy import asarray
from p2sim import ArmAnimatorApp

class MyArmSim(ArmAnimatorApp):
    def __init__(self,Tp2ws,**kw):
      ###
      ### Student team selection -- transform from workspace coordinates to world
      ###
      Tws2w = asarray([
           [1,0,0,  0],
           [0,1,0, -5],
           [0,0,1,-10],
           [0,0,0,  1]
      ])
      ###
      ### Arm specification
      ###
      armSpec = asarray([
        [0,0.02,1,5,0],
        [0,1,0,5,1.57],
        [0,1,0,5,0],
      ]).T
      ArmAnimatorApp.__init__(self,armSpec,Tws2w,Tp2ws,
        simTimeStep=0.1, # Real time that corresponds to simulation time of 0.1 sec
        **kw
      )

    def show(self,fvp):
      fvp.plot3D([0],[0],[0],'^k',ms=10) # Plot black triangle at origin
      return ArmAnimatorApp.show(self,fvp)

    def onStart(self):
      ArmAnimatorApp.onStart(self)
      ###
      ### TEAM CODE GOES HERE
      ###

    def onEvent(self,evt):
      ###
      ### TEAM CODE GOES HERE
      ###    Handle events as you see fit, and return after
      return ArmAnimatorApp.onEvent(self,evt)

if __name__=="__main__":
  # Transform of paper coordinates to workspace
  Tp2ws = asarray([
       [0.7071,0,-0.7071,0],
       [0,     1,      0,0],
       [0.7071,0, 0.7071,0],
       [0,     0,      0,1]
  ])
  Tp2ws=asarray([[  1.  ,   0.  ,   0.  ,   0.16],
       [  0.  ,   0.71,   0.71,   1.92],
       [  0.  ,  -0.71,   0.71,  10.63],
       [  0.  ,   0.  ,   0.  ,   1.  ]])
  app = MyArmSim(Tp2ws,
     ## Uncomment the next line (cfg=...) to save video frames;
     ## you can use the frameViewer.py program to view those
     ## frames in real time (they will not display locally)
     # cfg=dict(logVideo="f%04d.png")
    )
  app.run()
