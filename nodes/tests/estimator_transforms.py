#!/usr/bin/env python
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, TransformStamped
from gazebo_msgs.msg import ModelStates
import tf2_ros

def handle_apriltag_est(msg):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "pad"
    t.child_frame_id = "base_link_tag"
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    
    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w

    br.sendTransform(t)

def handle_vision_est(msg):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "pad"
    t.child_frame_id = "base_link_vis"
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    
    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w

    br.sendTransform(t)

def handle_gazebo_states(msg):
    lezl_id = msg.name.index("lezl")
    #p3at_id = msg.name.index("pioneer3at_lander")
    br = tf2_ros.TransformBroadcaster()
    tl = TransformStamped()
    tl.header.stamp = rospy.Time.now()
    tl.header.frame_id = "odom"
    tl.child_frame_id = "base_link_true"
    tl.transform.translation.x = msg.pose[lezl_id].position.x
    tl.transform.translation.y = msg.pose[lezl_id].position.y
    tl.transform.translation.z = msg.pose[lezl_id].position.z
    
    tl.transform.rotation.x = msg.pose[lezl_id].orientation.x
    tl.transform.rotation.y = msg.pose[lezl_id].orientation.y
    tl.transform.rotation.z = msg.pose[lezl_id].orientation.z
    tl.transform.rotation.w = msg.pose[lezl_id].orientation.w

    br.sendTransform(tl)

#    tp = TransformStamped()
#    tp.header.stamp = rospy.Time.now()
#    tp.header.frame_id = "odom"
#    tp.child_frame_id = "p3at_true"
#    tp.transform.translation.x = msg.pose[p3at_id].position.x
#    tp.transform.translation.y = msg.pose[p3at_id].position.y
#    tp.transform.translation.z = msg.pose[p3at_id].position.z
#    
#    tp.transform.rotation.x = msg.pose[p3at_id].orientation.x
#    tp.transform.rotation.y = msg.pose[p3at_id].orientation.y
#    tp.transform.rotation.z = msg.pose[p3at_id].orientation.z
#    tp.transform.rotation.w = msg.pose[p3at_id].orientation.w
#
#    br.sendTransform(tp)

if __name__ == "__main__":
    rospy.init_node("gazebo_tf")
    rospy.Subscriber("gazebo/model_states",ModelStates,handle_gazebo_states)
    rospy.Subscriber("vision_estimate",Odometry,handle_vision_est)
    rospy.Subscriber("apriltag_estimate",Odometry,handle_apriltag_est)
    rospy.spin()

