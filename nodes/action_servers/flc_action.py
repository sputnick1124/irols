#!/usr/bin/env python
import rospy
import actionlib

import irols.msg

from irols.utils import Controller
from yapflm import fisyaml

import tf2_ros
#from tf.transformations import euler_from_quaternion as efq
from numpy import dot, sqrt, sign, matrix
from numpy.linalg import norm

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class IROLSTimer(rospy.Timer):
    def __init__(self,*args,**kwargs):
        super(IROLSTimer,self).__init__(*args,**kwargs)
        self._shutdown = False    

    def shutdown(self):
        super(IROLSTimer,self).shutdown()
        self._shutdown = True

    def is_shutdown(self):
        return any([self._shutdown,not self.is_alive()])

class FuzzyController(Controller):
    def __init__(self,fis_list,scales):
        super(FuzzyController,self).__init__()
        self.fis_list = fis_list
        self.scales = scales

    def calc_control(self,*errs):
        controls = []
        for fis,scale,err in zip(self.fis_list,self.scales,errs):
            E = err.E if abs(err.E) <= 1 else sign(err.E)
            Ed = err.Ed if abs(err.Ed) <= 1 else sign(err.Ed)
            unscaled = fis.evalfis([E,Ed])
#            print(unscaled,err.E,err.Ed)
            controls.append(unscaled*scale)
        controls[-1] = 0 # ignore yaw input for now
        return controls

class FLCServer(object):
    _feedback = irols.msg.DoFLCFeedback()
    _result = irols.msg.DoFLCResult()

    def __init__(self,name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                irols.msg.DoFLCAction,
                                                execute_cb=self.execute,
                                                auto_start=False)

        self.tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.cov_norm = 0
        self.cov_sub = rospy.Subscriber('odometry/filtered',Odometry,self.odom_cb)
        self.vel_sp_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped',
                                      Twist,
                                      queue_size=1)
        # ghost topic to observe control without actually exerting it
        self.vel_sp_pub_sim = rospy.Publisher('irols/cmd_vel_sim',
                                      Twist,
                                      queue_size=1)

        fis_names = ['x_fis','y_fis','z_fis','T_fis']
        fis_dicts = []
        for name in fis_names:
            if rospy.has_param(name):
                fis_dicts.append(rospy.get_param(name))
            else:
                rospy.logwarn('{0}: {1} is not on param server'.format(self._action_name,name))
        if len(fis_dicts) != len(fis_names):
            rospy.logerr('{0}: not starting the server'.format(self._action_name))
            return
        fis_list = [fisyaml.fis_from_dict(fd) for fd in fis_dicts]
        self.flc = FuzzyController(fis_list,[1,1,1,1])
        
        self.abort_timer = None
        self.aborted = False

        self._as.start()
        rospy.loginfo('{0}: online'.format(self._action_name))

    def execute(self,goal):
        self.aborted = False
        try:
            err = self.tf_buffer.lookup_transform('pad','base_link_true',rospy.Time(0))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr('{0}: {1}'.format(self._action_name,e))
            self._as.set_aborted()
            return

        if goal.fis_array:
            fis_list = [fisyaml.fis_from_ros_msg(msg) for msg in goal.fis_array.fis_array.fis_array]
            self.flc.fis_list = fis_list
        rate = rospy.Rate(10)
        cmd_vel = Twist()
        pos_err = err.transform.translation
        dist =sqrt(dot([pos_err.x,pos_err.y,pos_err.z],[pos_err.x,pos_err.y,pos_err.z]))
        while dist > 0.5 and self._as.is_active():
            try:
                if self._as.is_preempt_requested():
                    self._as.set_preempted(self._result)
                    return
#                if not self._state.mode == "OFFBOARD" and count > 5:
#                    rospy.loginfo('{0}: setpoints are primed'.format(self._action_name))
#                    self.set_mode_client(custom_mode="OFFBOARD")
                err = self.tf_buffer.lookup_transform('pad','base_link_true',rospy.Time(0))
                #err = self.tf_buffer.lookup_transform('pad','base_link',rospy.Time(0))
                pos_err = err.transform.translation
                vx,vy,vz,dT = self.flc(err)
                cmd_vel.linear.x = vx
                cmd_vel.linear.y = vy
                cmd_vel.linear.z = vz
                cmd_vel.angular.z = dT
#                cmd_vel.linear.x = pos_err.x
#                cmd_vel.linear.y = pos_err.y
#                cmd_vel.linear.z = pos_err.z
                self.vel_sp_pub.publish(cmd_vel)

                dist =sqrt(dot([pos_err.x,pos_err.y,pos_err.z],[pos_err.x,pos_err.y,pos_err.z]))
                self._feedback.distance = dist
                self._as.publish_feedback(self._feedback)
                rate.sleep()
#                rospy.loginfo('{0}: norm(covariance) = {1}'.format(self._action_name,self.cov_norm))
                if (rospy.is_shutdown() or self.cov_norm > 0.125) and self._as.is_active():
                    if self.abort_timer is None or self.abort_timer.is_shutdown():
                        rospy.loginfo("{0}: covariance grew too large. Setting abort timer".format(self._action_name))
                        self.abort_timer = IROLSTimer(rospy.Duration(2),self.abort_cb,oneshot=True)
                elif self.abort_timer is not None and not self.abort_timer.is_shutdown():
                    rospy.loginfo("{0}: covariance improved within time limit. Resuming".format(self._action_name))
                    self.abort_timer.shutdown()
                elif self.aborted:
                    rospy.loginfo("{0}: aborted".format(self._action_name))
                    return
                        
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue
        rospy.loginfo("{0}: dist={1}, self._as._isactive()=".format(self._action_name,dist,self._as.is_active()))
        self._result.final_err.x = pos_err.x
        self._result.final_err.y = pos_err.y
        self._result.final_err.z = pos_err.z
        self._as.set_succeeded(self._result)

    def abort_cb(self,event):
        if self._as.is_active():
            rospy.loginfo("{0}: aborting".format(self._action_name))
            self.aborted = True
            self._as.set_aborted()
    
    def odom_cb(self,msg):
        cov = msg.pose.covariance
        cov_mx = matrix(cov).reshape((6,6))[:3,:3]
        self.cov_norm = norm(cov_mx)

if __name__ == "__main__":
    rospy.init_node('flc_action_server')
    server = FLCServer('flc_action')
    rospy.spin()
