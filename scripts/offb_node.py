#!/usr/bin/env python
import rospy
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

class LEZL(object):
    def __init__(self):
        rospy.init_node('lezl')
        
