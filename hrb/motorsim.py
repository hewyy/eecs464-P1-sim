from numpy import zeros, asarray, asfarray, concatenate
from numpy.random import randint
  
def clip(a,mn,mx):
    return mn if a<mn else (mx if a>mx else a) # Hybrid clip
    # return mn + (mx-mn)*tanh((a-mn)/(mx-mn)) # Smooth clip

def satFb( x, mx, dx ):
    return dx if abs(x)<mx or x*dx<0 else 0

TH, OM, BL, EI, TMP,TQ0,TQ1,TQ2,TQ3,TQD,POS,GV,GP,NDim = list(range(14))

class MotorModel( object ):
    def __init__(self):
        self.goalPos = 0 # [rad] Goal position to move to
        self.goalVel = 0 # [rad/sec] Goal velocity to use
        self.maxSpeed = 1 # [rad/sec] Limit on commanded speeds used by controller
        self.error = None # Current error condition
        ## PID gains
        self.Kp = 4. # [amp/rad]
        self.Kd = 3. # [amp/(rad/sec)]
        self.Ki = 1 # [amp/(rad*sec)]
        self.sat = 2.0 # [rad*sec] Integrator saturation
        self.tqLimit = 1. # [amp] Upper limit on torque (in amps)
        self.blMag = .1 # [rad] Gear backlash magnitude (rad)
        self.mu = .05 # [amp/(rad/sec)] dynamic friction
        self.inertia = 1. # [amp/(rad/sec**2)] motor inertia
        self.thrm = 0.1 # [amp**2/sec] thermal sink rate
        self.nolo = 2. # [rad/sec] No-load speed of motor
        self.cmax = 10. # [amp] Maximal drive power system can produce
        self.maxTemp = 7.5 # [internal units] Thermal shutdown temperature 85C
        self.maxLog = 50 # maximal number of points in the simulation log
        return self.clear()

    def clear(self,ics=None):
        self.clearError()
        y0 = zeros(TMP+1)
        if ics is not None:
          y0[:len(ics)] = ics
        self.y = [y0]
        self.t = [0]
        self._aux = {} # store for aux outputs

    def clearError(self):
        self.error = None
        
    def _ext(self, t, th):
        return 0

    def _flow(self,t,y,p=None):
        th, om, bl, ei, tmp = y
        de = om - self.goalVel
        if self.goalPos is not None:
            e = th - self.goalPos
            dei = satFb(ei,self.sat,e)
        else:
            e = 0.
            dei = satFb(ei,self.sat,de)
        tqc0 = -self.Kp * e - self.Kd * de - self.Ki * ei
        tqc1 = clip(tqc0,-self.tqLimit, self.tqLimit)
        # Correct torque (current) for Back EMF (generated current) and temp
        tqc2 = tqc1 - om/self.nolo
        tqc3 = clip( tqc2, -self.cmax, self.cmax )
        # Thermal shutdown
        if tmp > self.maxTemp:
          self.error = "Thermal error"
        # Under error conditions the motor shuts down
        if self.error is not None:
          tqc3 = 0
        # Disturbance torque
        if callable(self._ext):
          tqd = self._ext(t,th-bl)
        else:
          tqd = self._ext
        # Change in rotation speed includes torque and dynamic friction
        dom = (tqc3 - self.mu * om + tqd) / self.inertia
        ## Integrate blacklash
        dbl = satFb(bl,self.blMag,om)
        # Heat accumulates as a function of current squared
        dtmp = -tmp*self.thrm + tqc3*tqc3
        # Keep a copy of all auxillary outputs
        gp = self.goalPos if self.goalPos else 0
        self._aux = asfarray( (tqc0,tqc1,tqc2,tqc3,tqd,th-bl,self.goalVel,gp) )
        return asfarray( (om,dom,dbl,dei,dtmp) )

    def stepIter(self,h):
        """
        Iterator that breaks down the function evaluations of a
        one time-step with Runge-Kutta 4 integration (the classical method)
        INPUT:
          h -- float>0 -- duration of time-step
        """
        t0 = self.t[-1]
        y0 = self.y[-1]
        l = len(y0)
        y0 = y0[:TMP+1]
        tm = t0+h/2.
        t1 = t0+h
        st = t0,y0; yield True,st
        k1 = h*self._flow(*st)
        if l==TMP+1:
          self.y[-1] = concatenate([y0,self._aux])
        else:
          assert l == NDim
        st = tm,y0+k1/2.; yield True,st
        k2 = h*self._flow(*st)
        st = tm,y0+k2/2.; yield True,st
        k3 = h*self._flow(*st)
        st = t1,y0+k3; yield True,st
        k4 = h*self._flow(*st)
        self.y.append(y0+(k1+2*k2+2*k3+k4)/6.)
        self.t.append(t1)
        if len(self.t)>self.maxLog:
          self.t = self.t[-self.maxLog:]
          self.y = self.y[-self.maxLog:]
        yield False, (self.t[-1], self.y[-1])
        
    def step(self,h):
        for _,(t,y) in self.stepIter(h):
          pass
        return t,y

    def _get_ty(self):
        """ (INTERNAL)
        Get time history of the simulation
        """
        # Note: last data point never has aux; thus dropped
        return asarray(self.t[:-1]),asarray(self.y[:-1])
  
    def get_pos(self):
        """
        Obtain current position
        """
        return int((self.y[-1][TH]+self.y[-1][BL]) * 18000/3.14159) + randint(-200,200)
      
    def get_temp(self):
        """
        Obtain motor temperature (Centigrade)
        """
        return int(self.y[-2][TMP] * 8 + 25)
                   
    def get_error(self):
        """
        Obtain current error code
        """
        return self.error
      
    def set_pos(self,pos):
        """
        Set desired position / None to use velocity control only
        """
        if pos is None:
            self.goalPos = None
            return
        self.goalPos = pos * 3.14159 / 18000.

    def get_goal(self):
        """
        Get goal position / None if in velocity control mode
        """
        if self.goalPos is None:
            return None
        return int(self.goalPos * 18000/3.14159)
      
    def set_rpm(self,rpm):
        """
        """
        self.goalVel = rpm * 3.14159 /30.0
    
          
          
          
          
          
          
          