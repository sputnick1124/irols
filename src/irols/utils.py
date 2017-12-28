#!/usr/bin/env python
from tf.transformations import euler_from_quaternion as efq
from geometry_msgs.msg import Pose, PoseStamped

from math import sqrt, log
import numpy as np


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
        return self.calc_control(self.dx,self.dy,self.dz,self.dT)

    def calc_control(self,*errs):
        pass

def euclidean_distance(pose_a,pose_b,ignore_z=False):
    if isinstance(pose_a,PoseStamped):
        pose_a = pose_a.pose
    elif not isinstance(pose_a,Pose):
        raise TypeError('Type must be Pose[Stamped]')
    if isinstance(pose_b,PoseStamped):
        pose_b = pose_b.pose
    elif not isinstance(pose_b,Pose):
        raise TypeError('Type must be Pose[Stamped]')
    a = pose_a.position
    b = pose_b.position
    dx = b.x - a.x
    dy = b.y - a.y
    if ignore_z:
        dz = 0
    else:
        dz = b.z - a.z
    return sqrt(dx*dx + dy*dy + dz*dz)

def fitness_fcn(pos_err):
    """
    Compute the fitness of the landing attempt.

    Parameters:
        x_err, y_err, z_err: Time histories of error

    Returns:
        J: the fitness
    """
    dt = 0.001
    xy_dist = np.linalg.norm(pos_err[:,:2], axis=1)
    slant_range = np.linalg.norm(pos_err, axis=1)
    ix = slant_range > 0.1
    t = np.arange(0, dt*len(ix), dt)
    A = np.vstack([t, np.ones(len(ix))]).T
    slope, yint = np.linalg.lstsq(A, slant_range[ix])[0]
    slope = -abs(slope)

    log_inv_slant_range = np.log(1/slant_range)
    err_cost = (xy_dist**2)*(log_inv_slant_range - log_inv_slant_range.min())
    tm_cost = (slant_range - yint - slope*t)**2
    J = cost + tm_cost


