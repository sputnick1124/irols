#!/usr/bin/env python
import rospy
import actionlib

import irols.msg

from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry

from math import sqrt

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

class SeekServer(object):
    _feedback = irols.msg.DoSeekFeedback()
    _result = irols.msg.DoSeekResult()

    def __init__(self,name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                irols.msg.DoSeekAction,
                                                auto_start=False)
        self._as.register_goal_callback(self.goal_cb)
        self._as.register_preempt_callback(self.preempt_cb)

        rospy.wait_for_message('p3at/odom',Odometry)
        self.alt_sp = 10
        self.pos_sp = PoseStamped()
        self.odom_pub = rospy.Subscriber('p3at/odom',
                                         Odometry,
                                         self.handle_odom)
        self.curr_pos = rospy.wait_for_message('mavros/local_position/pose',
                                               PoseStamped).pose
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
                                              PoseStamped,
                                              self.handle_local_pose)
        self.state = rospy.wait_for_message('mavros/state',State)
        self.state_sub = rospy.Subscriber('mavros/state',
                                          State,
                                          self.handle_state)
        self.pos_sp_pub = rospy.Publisher('mavros/setpoint_position/local',
                                             PoseStamped,
                                             queue_size=3)
        
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode',
                                                  SetMode)

        self._as.start()
        rospy.loginfo('{0}: online'.format(self._action_name))
        self.execute_loop()
        


    def execute_loop(self):
        r = rospy.Rate(10)
        
        while euclidean_distance(self.curr_pos,self.pos_sp,ignore_z=True) > 0.00:
            if not self._as.is_active():
                continue
            if not self.state.mode == "OFFBOARD":
                self.set_mode_client(custom_mode="OFFBOARD")
            
            self.pos_sp.header.stamp = rospy.Time.now()
            self.pos_sp_pub.publish(self.pos_sp)
            self._feedback.distance = euclidean_distance(self.curr_pos,self.pos_sp)
            self._as.publish_feedback(self._feedback)
            r.sleep()
            if rospy.is_shutdown():
                if self._as.is_active():
                    self._as.set_aborted()
                return
        self._result.final_pos = self.curr_pos
        self._as.set_succeeded(self._result)
    
    def goal_cb(self):
        """implicitly sets any previous goal to preempted"""
        goal = self._as.accept_new_goal()
        self.alt_sp = goal.alt_sp
        rospy.loginfo('{0}: received goal: {1}'.format(self._action_name,goal))
        rospy.loginfo('{0}: current goal state is {1}'.format(self._action_name,self._as.is_active()))
    
    def preempt_cb(self):
        self._as.set_preempted(self._result)

    def handle_odom(self,data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        self.pos_sp.pose.position.x = x
        self.pos_sp.pose.position.y = y
        self.pos_sp.pose.position.z = self.alt_sp
            

    def handle_local_pose(self,data):
        self.curr_pos = data.pose

    def handle_state(self,data):
        self.state = data

if __name__ == '__main__':
    rospy.init_node('seek_action_server')
    server = SeekServer('seek_action')
    rospy.spin()
