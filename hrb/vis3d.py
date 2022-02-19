#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 31 04:11:41 2020

@author: shrevzen
"""

from numpy import asarray,ones,c_,inf

def hyperCube(d):
    """
    Iterator yielding the corners of the d-dimensional hypercube in binary
    order.
    """
    if d<1:
      return
    if d == 1:
      yield (0,)
      yield (1,)
      return
    for hc in hyperCube(d-1):
      yield hc+(0,)
      yield hc+(1,)
xyzCube = asarray(list(hyperCube(3)),float)
xyzCube = c_[xyzCube, ones(len(xyzCube))]

def edgeIndexIter(d):
    """
    Iterator yielding indices of hypercube edge vertices
    """
    for k in range(1<<d):
      for di in range(d):
        m = 1<<di
        ko = k ^ m
        if ko>k:
          yield ko,k
iCube = asarray(list(edgeIndexIter(3)),int)
iFace = asarray(list(edgeIndexIter(2)),int)

def plotVE( ax, v, ei, *arg, **kw ):
    """
    Plot edges between vertices.
    INPUT:
      ax -- 3D projection matplotlib axes object
      v -- N x 3 or N x 4 -- points in 3D
      ei -- edge iterable -- iterator producing indices of points to connect with lines
    """
    for k in ei:
      vi = v[k,...].T
      ax.plot3D(vi[0],vi[1],vi[2],*arg,**kw)
    #ax.set(xlabel='x',ylabel='y',zlabel='z')
    ax.axis('equal')

class FourViewPlot:
  def __init__(self, fig, f = inf):
    """
    Display four standard views in the specified figure.

    If specified, f gives focal distance; use inf for isometric
    """
    self.xyz = fig.add_subplot(221)
    self.xy = fig.add_subplot(224)
    self.xz = fig.add_subplot(222)
    self.zy = fig.add_subplot(223)
    self.f = f

  def __getattr__(self,attr):
    """
    Attempt to map all attributes to the axes.

    On getting a value, we get the xyz axes
    On calling a method, we call all four and collect the results

    >>> fvp.grid(1) # turns grid on in all the grids
    >>> fvp.figure # returns the figure holding the xyz axes
    """
    m = getattr(self.xyz,attr,None)
    if m is None:
      raise AttributeError("No axis attributes %r" % attr)
    if not callable(m):
      return m
    def _doAll(*arg,**kw):
      return [[
        getattr(self.xyz,attr)(*arg,**kw), getattr(self.xz,attr)(*arg,**kw)
        ],[
        getattr(self.zy,attr)(*arg,**kw), getattr(self.xy,attr)(*arg,**kw)
      ]]
    return _doAll

  def plot3D(self,x,y,z,*arg,**kw):
    """
    Plot 3D lines in all 4 views
    """
    # 0.8660 = cos(pi/6)
    x,y,z = asarray(x),asarray(y),asarray(z)
    u = (x-y)*0.866
    v = z*0.866-(x+y)/2
    sxyz = (x+y+z)/self.f+1
    sxz = y/self.f+1
    sxy = z/self.f+1
    szy = x/self.f+1
    return [[
        self.xyz.plot(u*sxyz,v*sxyz,*arg,**kw), self.xz.plot(x*sxz,z*sxz,*arg,**kw)
        ],[
        self.zy.plot(z*szy,y*szy,*arg,**kw), self.xy.plot(x*sxy,y*sxy,*arg,**kw)
        ]]

__all__=[FourViewPlot, xyzCube, iCube, iFace, plotVE]
