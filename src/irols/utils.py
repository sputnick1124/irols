#!/usr/bin/env python
from tf.transformations import euler_from_quaternion as efq

class StateError(object):
    def __init__(self):
        self.t = 0
        self.E = 0
        self.Ed = 0
        self.Ei = 0

    def __call__(self,value,t):
        dt = t - self.t
        if dt <= 0:
            self.t = t
            return
        self.Ed = (value - self.E)/dt
#       self.Ei += ((value + self.E)/2)*dt
        self.Ei += self.E*dt
        self.E = value
        self.t = t
#       rospy.loginfo('(dt,E,Ei,Ed)=(%0.4f,%0.4f,%0.4f,%0.4f)'%(dt,self.E,self.Ei,self.Ed))

class Controller(object):
    def __init__(self):
        self.dx = StateError()
        self.dy = StateError()
        self.dz = StateError()
        self.dT = StateError()

    def __call__(self,err):
        """
        Assume this is getting a tf2_ros transform
        """
        t = err.header.stamp.to_sec()
        dxyz = err.transform.translation
        o = err.transform.rotation
        _,_,T = efq([o.x,o.y,o.z,o.w])
        self.dx(dxyz.x,t)
        self.dy(dxyz.y,t)
        self.dz(dxyz.z,t)
        self.dT(T,t)
        return calc_control(self.dx,self.dy,self.dz,self.T)

    def calc_control(self,*errs):
        pass
