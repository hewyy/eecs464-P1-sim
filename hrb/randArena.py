#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 25 21:31:43 2020

@author: shrevzen
"""

from numpy import ( 
    block, zeros, ones, dot, asarray, newaxis, tanh
)
from numpy.linalg import svd
from numpy.random import randn, rand, permutation

def randRot(dim=3):
    """
    Random rotation of dimension dim
    """
    U,_,_ = svd(randn(dim,dim))
    return U

def randSE(scl=1.,dim=3):
    """
    Generate a random rigid body motion.
    Translation is generated from an isotropic Lorenz distribution,
    making it heavy tailed.
    
    INPUT:
      scl -- float -- (default 1) rescaling of position variability
      dim -- int -- (default 3) dimension of the space
    """
    return block([
      [randRot(dim),scl/randn(dim,1)],
      [zeros((1,dim)),1]
    ])

def randHomog(scl=1.,dim=2, fscl=1e-4):
    """
    Generate a random homography.
    
    INPUT:
      scl -- float -- (default 1) rescaling of position variability
      dim -- int -- (default 2) for a P2 to P2 homography
      fscl -- float -- (default 1) rescaling of focal length
    """
    S = randSE(scl,dim)
    # Make S[-2,-1]=1 and make the focal distance Lorenz
    S[-1,:-1] = randn(dim) * fscl
    S[-1,-1] = 1
    return S

def randConv(pts,N,beta=None):
    """
    Pick N random points in the convex hull of pts
    
    INPUT:
      pts -- M x shape... -- corners
      N -- int -- number of points to generate
      beta -- float>0 or None -- if not none, extremizes weights using a 
          sigmoid function to leave on beta of the weights significant
    OUTPUT: N x shape...
    """
    pts = asarray(pts)
    if beta is None:
      w0 = rand(pts.shape[0],N)
    else:
      w0 = 1-tanh(rand(pts.shape[0],N)/beta)
    w1 = w0 / w0.sum(0)[newaxis,...]
    return dot(pts.T,w1).T
  
def remapRef( corners, ref, scl=1. ):
    """
    Randomize reference corners
    
    INPUT:
      corners -- list of N int -- corner ID-s
      ref -- 3 x N -- array of N 2D corners in homogeneous coordinates
      
    OUTPUT: ncorners, nref
      ncorners -- list of N int -- randomly permuted corners list
      nref -- 3 x N -- transformed corners, permuted to match corners list
    """
    T = randSE(scl,dim=2)
    P = permutation(list(range(len(corners))))
    ncorners = list(asarray(corners)[P])
    nref = dot(ref,T.T)[P]
    return ncorners, asarray(nref,int)
      
def randArena( excorners, fixed, msg, **kw ):
    """
    Randomize arena
    
    INPUT:
      excorners -- list of N int -- excorner tag IDs
      fixed -- list of int -- tag IDs not to move
      msg -- dict( id --> 4x2 ) -- corners of tags, in the same format as
         robotSimulator.MSG_TEMPLATE
      **kw -- additional arguments are passed to randHomog
      
    OUTPUT: nmsg
      nmsg -- randomized copy of msg. Randomization does the following:
        (1) A random homography is applied to all tags
        (2) All non corner tags have their centers translated to a random point
          in the convex hull of the corner tags
    """
    # Collect keys and values in arrays
    k = asarray(list(msg.keys()))
    v0 = asarray(list(msg.values()))
    v = block([[v0,ones(v0[...,0].shape)[...,newaxis]]])
    # Generate a random homography and apply to all points
    T = randHomog(**kw)
    nv = dot(v,T.T)
    u = nv[...,:-1]/nv[...,[-1]]
    # Construct indicator array of corners
    f = zeros(len(k),bool)
    pk = dict(zip(k,range(len(k))))
    for fi in fixed:
      f[pk[fi]] = True
    # Move non-corners
    ctr = u.mean(1)[:,newaxis,:]
    nctr = randConv( 
              ctr[[pk[ci] for ci in corners],...], 
              asarray(~f,int).sum(), 0.3 
    )
    u[~f,...] = u[~f,...] - ctr[~f,...]+ nctr
    # Rebuild message
    nmsg = dict(zip(k,( list([list(r) for r in v]) for v in asarray(u,int))))
    return nmsg
    
if __name__=="__main__":
    from robotSimIX import DEFAULT_MSG_TEMPLATE as msg
    from waypointShared import (
        excorners,waypoints,
        DEFAULT_corners as corners,
        DEFAULT_ref as ref
        )
    from pylab import figure, show
    from sys import argv
    
    try:
      from randAreaOutput import __file__ as fp0
      fp = fp0.replace(".pyc",".py")
    except:
      fp0 = "-r *.pyc"
      fp = "~/pyckbot/hrb/randArenaOutput.py"
      
    print("""
# This output is to be used as an automatically generated source file
# Usage:
#   %s > %s
#   rm %s    
from numpy import array
    """ % (" ".join(argv),fp,fp0) )
    if len(argv)>1:
      from numpy.random import seed
      seed(int(argv[1]))
      print("randSeed = %s\n" % argv[1])
    # Use '-y' as second parameter to skip the plotting
    showIt = not (len(argv)>2 and argv[2].startswith("-y"))
      
    nc,nref = remapRef(corners, ref)
    print("corners = %r\n" % nc)
    print("ref = %r\n" % nref)
    nmsg = randArena(excorners, corners, msg)
    print("MSG_TEMPLATE = %r\n" % nmsg) 
    print("__all__=[randSeed,corners,ref,MSG_TEMPLATE]\n")
    
    if showIt:
      fig = figure(1)
      fig.clf()
      ax = fig.add_subplot(121)
      for nm,xy in zip(corners, ref):
        ax.text(xy[0],xy[1],str(nm))
        ax.plot([xy[0]],[xy[1]],'o')
      ax.set_title('Reference')
      ax.axis('equal')
      ax.grid(True)
      ax = fig.add_subplot(122)
      for nm,xy in zip(nc, nref):
        ax.text(xy[0],xy[1],str(nm))
        ax.plot([xy[0]],[xy[1]],'o')
      ax.set_title('Remapped')
      ax.axis('equal')
      ax.grid(True)
      
      fig = figure(2)
      fig.clf()
      ax = fig.add_subplot(121)
      waypoints=set(waypoints)
      w = []
      for nm,xy0 in msg.items():
        xy = asarray(xy0)
        cxy = xy.mean(0)
        ax.text(cxy[0],cxy[1],str(nm),ha='center',va='center')
        ax.plot(xy[[0,1,2,3,0],0],xy[[0,1,2,3,0],1],'.-')
        if nm in waypoints:
          w.append(cxy)
      w = asarray(w).T
      ax.plot(w[0],w[1],'--k',lw=2,alpha=0.3)
      ax.set_title('Reference')
      ax.axis('equal')
      ax.grid(True)
      ax = fig.add_subplot(122)
      w = []
      for nm,xy0 in nmsg.items():
        xy = asarray(xy0)
        cxy = xy.mean(0)
        ax.text(cxy[0],cxy[1],str(nm),ha='center',va='center')
        ax.plot(xy[[0,1,2,3,0],0],xy[[0,1,2,3,0],1],'.-')        
        if nm in waypoints:
          w.append(cxy)
      w = asarray(w).T
      ax.plot(w[0],w[1],'--k',lw=2,alpha=0.3)
      ax.set_title('Remapped')
      ax.axis('equal')
      ax.grid(True)
      
      show()
      
      
      
      
      
      
      
      
      
      