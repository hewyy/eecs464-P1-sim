"""
Run an example of a robot arm

This can be steered via inverse Jacobian, or positioned.
"""
from sys import version_info
from numpy import dot, asarray
from numpy.linalg import pinv
from pylab import (
  clf, show, gcf, plot, axis
)
from arm import Arm

a = Arm(asarray([[0,1,0,3],[0,0,1,3]]*3).T)
f = gcf()
ang = [0,0,0,0,0,0]
while 1:
    f.set(visible=0)
    clf()
    a.plotAll(ang)
    f.set(visible=1)
    show()
    print("Angles: ",ang)
    if version_info.major == 2:
        d = input("direction as list / angles as tuple?>")
    else:
        d = eval(input("direction as list / angles as tuple?>"))
    if type(d) == list:
        Jt = a.getToolJac(ang)
        ang = ang + dot(pinv(Jt)[:,:len(d)],d)
    else:
        ang[:len(d)] = d
