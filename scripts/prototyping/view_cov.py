#!/usr/bin/env python
import rospy

from nav_msgs.msg import Odometry

import numpy as np
from numpy.linalg import norm

def handle_odom(data):
    cov = data.pose.covariance
    cov_mx = np.matrix(cov).reshape((6,6))[:3,:3]
    print norm(cov_mx)

rospy.Subscriber('odometry/filtered',Odometry,handle_odom)

rospy.init_node('view_cov')
rospy.spin()

