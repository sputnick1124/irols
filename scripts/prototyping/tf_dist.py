#!/usr/bin/env python
import rospy
import tf2_ros

from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion as efq

rospy.init_node('tf_dist')

tf_dist_pub = rospy.Publisher('irols/tf_dist',Twist,queue_size=1)

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

while not rospy.is_shutdown():
    rate = rospy.Rate(10)
    dist = Twist()
    try:
        t = tfBuffer.lookup_transform('pad','base_link',rospy.Time(0))
        dist.linear.x = t.transform.translation.x
        dist.linear.y = t.transform.translation.y
        dist.linear.z = t.transform.translation.z
        o = t.transform.rotation
        dist.angular.x, dist.angular.y, dist.angular.z = efq([o.x,o.y,o.z,o.w])
        tf_dist_pub.publish(dist)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print e
        rate.sleep()
        continue

