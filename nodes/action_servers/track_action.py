#!/usr/bin/env python
import rospy
import actionlib

import irols.msg

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

class DoTrackServer(object):
    _feedback = irols.msg.DoTrackFeedback()
    _result = irols.msg.DoTrackResult()

    def __init__(self,name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                irols.msg.DoTrackAction,
                                                execute_cb=self.execute,
                                                auto_start=False)

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
        self.pos_sp_pub = rospy.Publisher('mavros/setpoint_position/local',
                                             PoseStamped,
                                             queue_size=3)

        self._as.start()
        rospy.loginfo('{0}: online'.format(self._action_name))


    def execute(self,goal):
        rospy.loginfo('{0}: received goal: {1}'.format(self._action_name,goal))
        self.alt_sp = goal.alt_sp if goal.alt_sp else self.alt_sp
        r = rospy.Rate(10)
        
        while euclidean_distance(self.curr_pos,self.pos_sp,ignore_z=True) > 2:
            self.pos_sp.header.stamp = rospy.Time.now()
            self.pos_sp_pub.publish(self.pos_sp)
            self._feedback.distance = euclidean_distance(self.curr_pos,self.pos_sp)
            self._as.publish_feedback(self._feedback)
            r.sleep()
        self._result.final_pos = self.curr_pos
        self._as.set_succeeded(self._result)

    def handle_odom(self,data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        self.pos_sp.pose.position.x = x
        self.pos_sp.pose.position.y = y
        self.pos_sp.pose.position.z = self.alt_sp

    def handle_local_pose(self,data):
        self.curr_pos = data.pose

if __name__ == '__main__':
    rospy.init_node('track_action_server')
    server = DoTrackServer('track_action')
    rospy.spin()
