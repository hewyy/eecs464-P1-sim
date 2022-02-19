#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 23 01:14:25 2020

@author: shrevzen
"""
from time import time
from numpy import (
    asarray, stack, ones, identity, dot, newaxis, cumsum, c_, nan, inf
)
from numpy.linalg import inv
from arm import Arm, jacobian_cdas
from motorsim import MotorModel,TH,BL
from joy.plans import AnimatorPlan
from joy import JoyApp
from joy.decl import KEYDOWN, K_q, K_ESCAPE, progress
from vis3d import FourViewPlot, xyzCube, iCube, iFace, plotVE
from joy.misc import requiresPyGame
requiresPyGame()

class MassArm(Arm):
    def __init__(self,wl):
      return self.setup(wl)

    def setup(self,wl):
      Arm.setup(self,wl)
      CoM = [ asarray([0,0,0,1]) ] # Center of mass
      m = ones(self.geom[1].shape[1]) # Mass distribution
      M = [ 0 ] # Mass
      I = [ identity(3) ] # Geometric inertia (I/m)
      for gn,l in zip(self.geom[1:],wl[3]):
        # Mass distribution - baseline, plus linear with segment length
        m[:] = (3.+5*l)/m.size
        # CoM position
        M.append(sum(m))
        com = dot(gn,m)/M[-1]
        CoM.append(com)
        # Mass position offsets relative to CoM
        ofs = gn - com[:,newaxis]
        # Inertia matrix
        I.append(dot(m[newaxis,:]*ofs,ofs.T)/M[-1])
      # Link masses and inertias
      self.CoM = asarray(CoM)
      self.M = asarray(M)
      self.I = asarray(I)
      # Gravity torque is the gradient of gravitational potential energy
      doc = self.getGravityTorque.__doc__
      self.getGravityTorque = jacobian_cdas(
        lambda ang : asarray([self.getEgp(self.at(ang))]),
        ones(self.tw.shape[0])*0.05
      )
      self.getGravityTorque.__doc__=doc

    def getCoMs( self, A ):
      """
      Find the CoM-s of all segments

      INPUT:
        A -- list of transformations -- output of .at()

      OUTPUT: n x 3
        Array of CoM-s of all segments
      """
      return asarray([dot(a,com) for a,com in zip(A,self.CoM)])

    def getEgp( self, A ):
      """
      Get gravitational potential energy associated with a configuration
      """
      return dot(self.M,self.getCoMs(A)[:,2])

    def getGravityTorque(self,ang):
      """
      Compute the torque exerted by gravity on each of the joints (up to scale)

      INPUT:
        ang -- N -- joint angles

      OUTPUT: 1 x N
        torque on each of the joints
      """
      raise RuntimeError("should be overriden by constructor")

    def getFrzI( self, A ):
      """
      Compute the frozen chain inertia for all segments.
      The "frozen chain inertia" is the inertia of the remainder
      of the kinematic chain if all joints were locked.

      INPUT:
        A -- list of SE(3) -- output of .at()

      OUTPUT:
        list of I matrices -- the inertia matrix of the frozen chain relative to the center of mass
      """
      # Compute inverse transforms
      iA = [inv(a) for a in A]
      # Transform all inertias to world frame
      #  Each geometric inertia needs to be multiplied by the
      #  mass of the segment, and have its points transformed
      Ik = asarray([ dot(dot(a,ii*m),a.T)
             for a,ii,m in zip(A,self.I,self.M) ])
      # Cumulative inertia of the trailing segments from k out
      Ic = cumsum(Ik[::-1,...],0)[::-1]
      # Computer frozen inertia from each segment out
      fI = [ dot(dot(ia,ic),ia.T) for ia,ic in zip(iA,Ic) ]
      return fI

    def plot3D(self,A,ax):
      Arm.plot3D(self,A,ax)
      cx,cy,cz,_ = self.getCoMs(A).T
      ax.plot3D(cx,cy,cz,'ow')
      ax.plot3D(cx,cy,cz,'+k')

class ArmSim(MassArm):
    def __init__(self,wlc):
        wlc = asarray(wlc)
        assert wlc.ndim == 2 and wlc.shape[0] == 5
        MassArm.__init__(self,wlc[:-1])
        self.m = []
        for k in range(wlc.shape[1]):
          mm = MotorModel()
          # Set initial angle
          mm.clear([wlc[4,k]])
          # Set initial goal angle to match
          mm.set_pos(mm.get_pos())
          # Store in arm config
          self.m.append(mm)
        # Set joint compliance
        self.c = ones(len(self.m))

    def __iter__(self):
        for mm in self.m:
          yield mm

    def __getitem__(self,idx):
        return self.m[idx % len(self.m)]

    def __len__(self):
        return len(self.m)

    def step(self,h):
        """
        Do an integration step for the whole arm.

        THEORY OF OPERATION:
          This is a rather tricky thing to do, because the RK integrator
        requires multiple function evaluations per time point. To support this,
        each motor model has an internal iterator that generates these quadrature
        points. Thus, we call next() on all the motor models to obtain their
        quadrature points, then compute the coupling term (gravity torque),
        set it up for them, and let them compute the next quadrature point,
        until we are done. Then we return the results.
        """
        its = [ m.stepIter(h) for m in self.m ]
        while True:
          # Collect the next RK quadrature point
          #   we assume the cont value and timestamp will agree between motors
          #   so we only collect that from the first motor
          cont,(t,y0) = next(its[0])
          y = [y0]+[ next(mi)[1][1] for mi in its[1:] ]
          ang0 = asarray([ yi[TH]+yi[BL] for yi in y]) # motor angles
          if not cont:
            break
          tq = 3e-3*self.getGravityTorque(ang0).squeeze() # gravity torque on motors
          for mi,tqi in zip(self.m,tq): # push into motor objects
            mi._ext = -tqi
        ang1 = ang0 - self.c*tq # sagged angles
        return t,ang1,stack(y,1)

class ArmAnimatorApp( JoyApp ):
    def __init__(self,wlc,Tws2w,Tp2ws,simTimeStep=0.1,*arg,**kw):
      if 'cfg' not in kw:
        kw['cfg'] = {}
      kw['cfg'].update(windowSize = [1200, 800])
      self.simTS = simTimeStep
      JoyApp.__init__(self,*arg,**kw)
      progress("Simulation time: %g sec = 0.1 sec simulated" % self.simTS)
      self.arm = ArmSim(wlc)
      self.Tp2w = dot(Tws2w,Tp2ws)
      # World to paper
      self.Tw2p = inv(self.Tp2w)
      # Paper with origin at origin
      self.paper_p = asarray([[8,11,-1/2.56,1]])*xyzCube
      self.paper_w = dot(self.paper_p,self.Tp2w.T)
      # Workspace box
      L = 12 # Workspace in arm units
      self.ws_w = dot(asarray([[L,L,L,1]])*xyzCube,Tws2w.T)
      self.ws_p = dot(self.ws_w,self.Tw2p.T)
      # Projection onto paper in world
      self.Tprj = dot(self.Tp2w,asarray([[1,1,0,1]]).T*self.Tw2p)
      # World to relative paper (i.e. paper is unit cube)
      self.Tw2rp = self.Tw2p / self.paper_p[-1][:,newaxis]

    def _integrate(self):
      last = self.T0
      dt = self.simTS
      qi = None
      ti = None
      while True:
        # Run in units of dt
        now = self.now
        h = now - last
        if h<dt and ti:
          yield
          continue
        last = now
        ti,qi,yi =self.arm.step(h*0.1/dt) # Speedup ratio into simulation time
        # Store simulation time-step
        self.t.append(ti)
        self.q.append(qi)
        self.y.append(yi)
        pen = self.arm.getTool(qi)
        self.p.append(pen)
        # Line marking point is lowpass version of pen
        if self.l:
          self.l.append(self.l[-1] * 0.6 + pen * 0.4)
        else:
          self.l.append(pen)

    def _show(self,fvp):
        """
        Wrapper for show(), to allow subclasses to override
        """
        fvp.cla()
        fvp.set(xticks=[],yticks=[])
        return self.show(fvp)

    def show(self,fvp):
        ti = self.t[-1]
        qi = self.q[-1]
        pen = self.p[-1]
        li = self.l[-1]
        # self.arm.plot3D(self.arm.at(qi),fvp)
        sk = self.arm.getSkel(self.arm.at(qi)).T
        fvp.plot3D(sk[0],sk[1],sk[2],lw=3,marker='o',color='#808080')
        # Find current pen point in paper coordinates
        qq = dot(self.Tw2rp,pen)
        # If in sheet
        if (qq[0]>0) and (qq[0]<1) and (qq[1]>0) and (qq[1]<1):
          if qq[2]>1: # Pressed too far in
            lt = dict(marker='o',color='r')
          elif qq[2]>0: # Drawing
            lt = dict(marker='.',color='g')
          else: # Above sheet
            lt = dict(marker="+", ms=15, color="m")
        else: # Not above or below sheet
            lt = dict(marker="+",color="b")
        # Project onto paper and bring back to world coordinates
        wp = dot(self.Tprj,pen)
        fvp.plot3D([wp[0]],[wp[1]],[wp[2]],**lt)
        wl = dot(self.Tprj,li)
        fvp.plot3D([wl[0]],[wl[1]],[wl[2]],'xk')
        fvp.xyz.set_title("t = %.2f, n=%d" % (ti,len(self.t)))
        plotVE(fvp,self.paper_w,iCube,'g--',alpha=0.3)
        plotVE(fvp,self.paper_w[::2,:],iFace,'g-')
        plotVE(fvp,self.ws_w,iCube,'k:')

    def _animation(self, fig):
      fig.clf()
      last = self.T0
      fvp = FourViewPlot(fig,f=80.)
      sim = self._integrate()
      dt = 0.1
      while True:
        next(sim) # allow simulation to run
        # Make sure graphics run at most 33% of the time
        now = self.now
        if now-last < dt * 2:
          yield
          continue
        last = now
        # Draw graphics, measure execution time
        tic = time()
        self._show(fvp)
        dt = time() - tic
        # Display progress
        progress(("(%4.2f) " % dt) +
            " ".join(["%15s" % (
            "%d/%d/%s" % (m.get_pos(),m.get_goal(),
              (str(int(m.get_temp())) if m.get_error() is None else "*ERR*"))
            ) for m in self.arm
        ]),sameLine=True)
        yield

    def onStop(self):
      # We need this kind of import so as not to mess up JoyApp plotting
      from pylab import figure,savefig
      # Collect pen motions and tool tip motions
      p = asarray(self.p).T
      l = asarray(self.l).T
      t = asarray(asarray(self.t)*100,int)
      # Convert to paper coordinates
      qq = dot(self.Tw2rp,p)
      lp = dot(self.Tw2p,l)
      # Prepare output as integers:
      #  time, x, y, depth
      pout = c_[t,asarray(lp[:2]*100,int).T,asarray(qq[2]*100,int)]
      with open("result-%d.csv" % time(),"w") as rf:
        for pp in pout:
          rf.write(repr(list(pp))[1:-1]+"\n")
      # Draw on "paper"
      fig = figure(2)
      fig.clf()
      ax = fig.gca()
      # projected track is pale blue
      ax.plot(lp[0],lp[1],'-b',alpha=0.2)
      # Find when pen was up, and remove those points
      up = (qq[2]<0) | (qq[0]<0) | (qq[0]>1) | (qq[1]<0) | (qq[1]>1)
      lp[:,up]=nan
      # Draw in black
      ax.plot(lp[0],lp[1],'-k',lw=2)
      # Find when pen was jammed in paper
      bad = qq[2]>1
      if any(bad): # If found, highlight in red
        lp[:,~bad] = nan
        ax.plot(lp[0],lp[1],'.-r',lw=3,alpha=0.5)
      # Draw the frame of the paper
      fr = self.paper_p[::2,[0,1]][[0,1,3,2,0]].T
      ax.plot(fr[0],fr[1],'b--',lw=2)
      ax.axis('equal')
      ax.grid(1)
      savefig("result-%d.png" % time(),dpi=300)

    def onStart(self):
      """
      Start the JoyApp and the simulation
      """
      self.ani = AnimatorPlan(self,self._animation)
      self.t,self.q,self.y,self.p,self.l = [],[],[],[],[]
      self.T0 = self.now
      self.ani.start()

    def onEvent(self,evt):
      """
      The keyboard row: asdfghjkl moves your motors one way
      The keyboard row: zxcvbnm,. moves your motors the other way
      'q' will quit and store the results in a results.png image and results.csv
      file.
      """
      # Ignore everything except keydown events
      if evt.type != KEYDOWN:
        return
      # Punt exit keys
      if evt.key in {K_q, K_ESCAPE}:
        return JoyApp.onEvent(self,evt)
      # row of 'a' on QWERTY keyboard increments motors
      p = "asdfghjkl".find(evt.unicode)
      if p>=0:
        self.arm[p].set_pos(self.arm[p].get_goal() + 500)
        return
      # row of 'z' in QWERTY keyboard decrements motors
      p = "zxcvbnm,.".find(evt.unicode)
      if p>=0:
        self.arm[p].set_pos(self.arm[p].get_goal() - 500)
        return

if __name__=="__main__":
  raise RuntimeError("This is not a script. Try 'myarmsim.py'")
