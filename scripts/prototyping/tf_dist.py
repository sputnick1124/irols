#!/usr/bin/env python
import rospy
import tf2_ros

rospy.init_node('tf_dist')

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

while not rospy.is_shutdown():
    rate = rospy.Rate(10)
    try:
        t = tfBuffer.lookup_transform('pad','base_link',rospy.Time(0))
        print t
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print e
        rate.sleep()
        continue

