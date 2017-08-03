#!/usr/bin/env python
import rospy
import actionlib

import yapflm.fisyaml
from utils import Controller

import irols.msg

import tf2_ros
from numpy import dot, sqrt

from geometry_msgs.msg import Twist

class FuzzyController(Controller):
    def __init__(self,fis_list,scales):
        super(FuzzyController,self).__init__()
        self.fis_list = fis_list
        self.scales = scales
    
    def calc_control(self,*errs):
        controls = []
        for fis,scale,err in zip(self.fis_list,self.scales,errs):
            controls.append(fis.evalfis(err.E,err.Ed)*scale)
        return controls

class DoFLCServer(object):
    _feedback = irols.msg.DoFLCFeedback()
    _result = irols.msg.DoFLCResult()
    
    def __init__(self,name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                irols.msg.DoTrackAction,
                                                execute_cb=self.execute,
                                                auto_start=False)

        tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(tf_buffer)

        self.vel_sp_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped',
                                      Twist,
                                      queue_size=1)

        xy_dict = rospy.get_param('xy_fis')
        z_dict = rospy.get_param('z_fis')
        T_dict = rospy.get_param('T_fis')
        fis_list = [fisyaml.fis_from_dict(fd) for fd in [xy_dict,xy_dict,z_dict,T_dict]]
        self.flc = FuzzyController(fis_list,[3,3,4,2])

        self._as.start()
        rospy.loginfo('{0}: online'.format(self._action_name))

    def execute(self,goal):
        try:
            err = self.tf_listener.lookup_transform('pad','base_link',rospy.Time(0))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr('{0}: {1}'.format(self.__action_name,e))
            self._as.set_aborted()
            return
        
        rate = rospy.Rate(10)
        cmd_vel = Twist()
        dist =sqrt(dot([err.x,err.y,err.z],[err.x,err.y,err.z])) 
        while dist > 0.3:
            err = self.tf_listener.lookup_transform('pad','base_link',rospy.Time(0))
            vx,vy,vz,dT = self.flc(err.x,err.y,err.z,err.T)
            cmd_vel.linear.x = vx
            cmd_vel.linear.y = vy
            cmd_vel.linear.z = vz
            cmd_vel.angular.z = dT
            self.vel_sp_pub.publish(cmd_vel)
            
            dist =sqrt(dot([err.x,err.y,err.z],[err.x,err.y,err.z]))
            self._feedback.dist = dist
            self._as.publish_feedback(self._feedback)
            rate.sleep()
        
        self._result.final_err.x = err.x
        self._result.final_err.y = err.y
        self._result.final_err.z = err.z
        self._as.set_succeded(self._result)

if __name__ == "__main__":
    rospy.init_node('flc_action_server')
    server = DoFLCServer('flc_action')
    rospy.spin()
