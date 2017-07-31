#!/usr/bin/env python
import rospy

from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

from tf.transformations import quaternion_matrix as qm

import numpy as np

class Comparator(object):
    def __init__(self):
        self.odom_sub = rospy.Subscriber('vision_estimate',
                                         Odometry,
                                         self.handle_odom)
        self.model_sub = rospy.Subscriber('gazebo/model_states',
                                          ModelStates,
                                          self.handle_states)
        self.err_pub = rospy.Publisher('estimate_error',
                                       Pose,
                                       queue_size=10)
        self.err = Pose()
        self.est = Pose()
        self.true = Pose()

    def handle_odom(self,data):
        self.est = data.pose.pose
    
    def handle_states(self,data):
        p3at = data.name.index('pioneer3at_lander')
        lezl = data.name.index('lezl')
        l_true = data.pose[lezl]
        p_true = data.pose[p3at]
        o = l_true.orientation
        Rw_b = np.matrix(qm([o.x,o.y,o.z,o.w]))[:3,:3]
        y = -(l_true.position.x-p_true.position.x)
        x = l_true.position.y-p_true.position.y
        z = l_true.position.z-p_true.position.z
        x,y,z = Rw_b.T*np.matrix([x,y,z]).T
        self.true.position.x = float(x)
        self.true.position.y = float(y)
        self.true.position.z = float(z)
    
    def run(self):
        rate = rospy.Rate(10)
        rospy.wait_for_message('vision_estimate',Odometry)
        while not rospy.is_shutdown():
            self.err.position.x = self.est.position.x - self.true.position.x
            self.err.position.y = self.est.position.y - self.true.position.y
            self.err.position.z = self.est.position.z - self.true.position.z
            self.err_pub.publish(self.err)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node('error_estimator')
    comp = Comparator()
    comp.run()
