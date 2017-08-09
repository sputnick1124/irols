#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class Pioneer(object):
	def __init__(self):
		self.pub = rospy.Publisher('/p3at/cmd_vel',Twist,queue_size=1,
					latch=True)
		rospy.init_node('pioneer_rover')
		rospy.on_shutdown(self.stop_rover)

	def move_rover(self,lin=0.5,ang=-0.25):
		vel = Twist()
		vel.linear.x = lin
		vel.angular.z = ang
		self.pub.publish(vel)

	def stop_rover(self):
		self.move_rover(0,0)

if __name__ == "__main__":
	pioneer = Pioneer()
	try:
		pioneer.move_rover()
		print("Moving rover")
		rospy.spin()
	except rospy.ROSInterruptException:
		print("Hmmm")
		pass


