#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 21 01:42:48 2020

@author: shrevzen
"""
from sys import argv

from numpy.random import rand, randint, seed

if len(argv)>1:
  rs = int(argv[1])
  seed(rs)
  print(f"# Seed used {rs}")
else:
  rs = None
  
from numpy import (
    asfarray, asarray, dot, cross, newaxis, eye, c_, allclose, sqrt
    )

from vis3d import xyzCube, iCube, plotVE, FourViewPlot, iFace

from pylab import figure, show

def norm(x):
  x = asfarray(x)
  return sqrt((x*x).sum())

def z2R(z,xHint = [1,0,0]):
  """
  Create rotation that takes the z axis to z, and has x aligned with hint
  as much as possible
  """
  z = asarray(z)
  z /= norm(z)
  x = dot(eye(3)-(z[:,newaxis] @ z[newaxis,:]), xHint)
  assert not allclose(x,0)
  x /= norm(x)
  y = cross(z,x)
  return c_[x,y,z]

# List of allowed normal vectors
Z = asfarray([[0,0,1],[0,0,1],[0,1,1],[1,0,1],[1,1,0],[1,1,1]])
# Pick random allowed rotation matrix
R = z2R( Z[randint(len(Z))], [1,0,0] if rand()>0.5 else [0,1,0] ) 
# Size of box
L = 12
# Workspace box
wso = -L/2*asfarray([[1,1,1,0]])
ws = asfarray([[L,L,L,1]])*xyzCube+wso
# Paper in native coordinates
Lx = 8
Ly = 11
paper_p = (asfarray([[Lx,Ly,-1/2.56,1]])*xyzCube)

# Rotate paper points
pr = dot(paper_p[:,:-1],R.T).T
mn = pr.min(1) # Bounding box mins
mx = pr.max(1) # Bounding box maxs
scl = L-(mx-mn) # Slack
ofs = rand(3) * scl - mn # compute uniform random offset in BBox

# Build SE(3) matrix
Tp2ws = eye(4)
Tp2ws[:3,:3] = R
Tp2ws[:-1,-1] = ofs
Tp2ws = Tp2ws.round(2)

print("Tp2ws=%r" % Tp2ws)
if rs is not None:
  with open("p2specs.py","a+") as fi:
    fi.write(f"\n#seed {rs}\n")
    fi.write("Tp2ws=%r\n" % Tp2ws)
    
# Build x,y,s

s = rand()*4+2
x = rand()*(Lx - s)
y = rand()*(Ly - s)
sq_p = xyzCube[::2,:] * [[s,s,0,1]] + [[x,y,0,0]]
sq = dot(sq_p,Tp2ws.T)+wso

print("x,y,s = %.1g,%.1g,%.1g" % (x+s/2,y+s/2,s/2))
if rs is not None:
  with open("p2specs.py","a+") as fi:
    fi.write("x,y,s = %.1g,%.1g,%.1g\n" % (x+s/2,y+s/2,s/2))

# Apply to "paper"
Tp2ws[:,[-1]] += wso.T
paper_w = dot(paper_p,Tp2ws.T)
fig = figure(1)
fig.clf()
fvp = FourViewPlot(fig,100.)
plotVE(fvp,paper_w,iCube,'g--',alpha=0.3)
plotVE(fvp,paper_w[::2,:],iFace,'g-')
plotVE(fvp,ws,iCube,'k:')
plotVE(fvp,sq,iFace,'r-')

if rs is not None:
  fig.savefig("p2specs-%d.png" % rs)




