#!/usr/bin/env python
import rospy
import actionlib

import irols.msg

from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Pose

from math import sqrt

def euclidean_distance(pose_a,pose_b):
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
    dz = b.z - a.z
    return sqrt(dx*dx + dy*dy + dz*dz)

class WPServer(object):
    _feedback = irols.msg.DoWPFeedback()
    _result = irols.msg.DoWPResult()

    def __init__(self,name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                irols.msg.DoWPAction,
                                                execute_cb=self.execute,
                                                auto_start=False)
        self._curr_pose = rospy.wait_for_message('mavros/local_position/pose',
                                                 PoseStamped)
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode',SetMode)
        self._state = rospy.wait_for_message('mavros/state',
                                             State)
        self.state_sub = rospy.Subscriber('mavros/state',
                                     State,
                                     self.handle_state)
        self.pose_sub = rospy.Subscriber('mavros/local_position/pose',
                                     PoseStamped,
                                     self.handle_pose)
        self.pos_sp_pub = rospy.Publisher('mavros/setpoint_position/local',
                                          PoseStamped,
                                          queue_size=10)
        self._as.start()
        rospy.loginfo('{0}: online'.format(self._action_name))

    def execute(self, goal):
        rospy.loginfo('{0}: received goal: {1}'.format(self._action_name,goal))
        goal_pose = goal.goal_pos
        init_pose = self._curr_pose
        goal_pose_stamped = PoseStamped(pose=goal_pose)
        radius = goal.radius if goal.radius else 0.2
        r = rospy.Rate(10)
        
        init_dist = euclidean_distance(init_pose,goal_pose)
        count = 0
        while euclidean_distance(self._curr_pose,goal_pose) > radius:
            goal_pose_stamped.header.stamp = rospy.Time.now()
            self.pos_sp_pub.publish(goal_pose_stamped)
            
            if not self._state.armed:
                rospy.logwarn('{0}: vehicle is not armed. Aborting'.format(self._action_name))
                self._result.final_pos = self._curr_pose.pose
                self._as.set_aborted(self._result)
                return
                
            if not self._state.mode == "OFFBOARD" and count > 5:
                rospy.loginfo('{0}: setpoints are primed'.format(self._action_name))
                self.set_mode_client(custom_mode="OFFBOARD")
            
            if self._as.is_preempt_requested():
                rospy.loginfo('{0}: preempted. Exiting'.format(self._action_name))
                self._as.set_preempted()
                return
            
            self._feedback.percent_complete = euclidean_distance(self._curr_pose,init_pose)/init_dist
            self._as.publish_feedback(self._feedback)
            count += 1
            r.sleep()
        rospy.loginfo('{0}: waypoint reached'.format(self._action_name))
        self._result.final_pos = self._curr_pose.pose
        self._as.set_succeeded(self._result)

    def handle_pose(self, data):
        self._curr_pose = data
        
    def handle_state(self, data):
        rospy.logdebug('{0}: current state is {1}'.format(self._action_name,data))
        self._state = data

if __name__ == '__main__':
    rospy.init_node('wp_action_server')
    server = WPServer('wp_action')
    rospy.spin()
