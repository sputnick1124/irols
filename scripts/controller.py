#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from mavros_msgs.srv import CommandBool, CommandTOL

class Controller(object):
	def __init__(self):
		rospy.init_node('controller')
		self.timer = None
		self.state_sub = rospy.Subscriber('/lezl/state',TwistStamped,self.calc_error)
		self.landed_pub = rospy.Publisher('lezl/landed',Bool,queue_size=10)
		self.controller_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped,queue_size=10)
		self.active_sub = rospy.Subscriber('/tracker/tracked',Bool,self.activate)
		self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
		self.dx = self.StateError()
		self.dy = self.StateError()
		self.dz = self.StateError()
		self.dT = self.StateError() # theta
		# setpoints
		self.x_s = 0
		self.y_s = 0
		self.z_s = 0.25
		self.T_s = 0
		self.effort = TwistStamped()
		self.rate = rospy.Rate(20)
		self.active = False
		self.run()
		rospy.spin()

	def land(self):
		self.arming_client(False)

	def run(self,timerevent=None):
		self.t = rospy.Time.now()
		if self.timer:
			self.timer.shutdown()
		while not rospy.is_shutdown():
			if (rospy.Time.now() - self.t) > rospy.Duration(5):
				warn = "No state update for at least %f secs. Pausing controller." % 5
				rospy.loginfo(warn)
				self.timer = rospy.Timer(rospy.Duration(5),self.run) #will call run() every 5 secs
				return
			elif not self.active:
				self.rate.sleep()
				continue
			self.effort.twist = self.calc_effort()
			self.effort.header.stamp = rospy.Time.now()
			self.controller_pub.publish(self.effort)
			self.rate.sleep()

	def activate(self,data):
		self.active = data.data

	def calc_error(self,data):
		t = data.header.stamp
		pos = data.twist.linear
		dx, dy, dz = (-(self.x_s - pos.x),-(self.y_s - pos.y),(self.z_s - pos.z))
		if abs(dz) <= self.z_s:
			self.land()
			self.landed_pub.publish(True)
			sys.exit(0)
		ang = (self.T_s - data.twist.angular.z)
		self.dx(dx,t.to_sec())
		self.dy(dy,t.to_sec())
		self.dz(dz,t.to_sec())
		self.dT(ang,t.to_sec())
		self.t = t

	class StateError(object):
		def __init__(self):
			self.t = 0
			self.E = 0
			self.Ed = 0
			self.Ei = 0

		def __call__(self,value,t):
			dt = t - self.t
			rospy.loginfo('dt = %f'%dt)
			self.Ed = (value - self.E)/dt
			self.Ei += ((value + self.E)/2)*dt
			self.E = value
			self.t = t

class PID(Controller):
	def __init__(self,Kp,Kd,Ki,u_max):
		#each is a list/tuple (one for each state)
		self.Kp = Kp
		self.Kd = Kd
		self.Ki = Ki
		self.u = u_max
		super(PID,self).__init__()

	def calc_effort(self):
		effort = Twist()
		effort.linear.x = self.combine(self.dy,0)
		effort.linear.y = -self.combine(self.dx,1)
		effort.linear.z = self.combine(self.dz,2)
		effort.angular.z = self.combine(self.dT,3)
		return effort

	def combine(self,error,index):
		u = error.E*Kp[index] + error.Ed*Kd[index] + error.Ei*Ki[index]
		return u if abs(u) <= self.u[index] else u/abs(u)*self.u[index]


if __name__ == "__main__":
#	Kp = (0.8,0.8,0.8,10)
	Kp = (5,5,5,10)
	Kd = (0.2,0.2,0.1,0.3)
	Ki = (0.0002,0.0002,0.0001,0.0003)
	Kd = (0,0,0,0)
#	Ki = (0,0,0,0)
	u_max = (3,3,1.2,5)
	try:
		PID(Kp,Kd,Ki,u_max)
	except rospy.ROSInterruptException:
		pass
