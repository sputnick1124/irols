#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from random import uniform

class Pioneer(object):
	def __init__(self):
		self.pub = rospy.Publisher('/p3at/cmd_vel',Twist,queue_size=1,
					latch=True)
		rospy.init_node('pioneer_rover')
		rospy.on_shutdown(self.stop_rover)

	def move_rover(self,lin,ang):
		vel = Twist()
		vel.linear.x = lin
		vel.angular.z = ang
		self.pub.publish(vel)

	def stop_rover(self):
		vel = Twist()
		vel.linear.x = 0
		vel.angular.z = 0
		self.pub.publish(vel)

if __name__ == "__main__":
	pioneer = Pioneer()
	try:
		rate = rospy.Rate(1)
		print("Moving rover")
		while not rospy.is_shutdown():
			lin = 0.5
			ang = uniform(-1,1)
			pioneer.move_rover(lin,ang)
			rate.sleep()
	except rospy.ROSInterruptException:
		print("Hmmm")
		pass


