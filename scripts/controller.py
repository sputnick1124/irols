#!/usr/bin/env python
from __future__ import division
import rospy
import sys, os
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, TwistStamped, Vector3, Vector3Stamped
from mavros_msgs.srv import CommandBool, CommandTOL

from dynamic_reconfigure.server import Server
from irols.cfg import pid_gainsConfig

from yapflm import FIS
from fisparse import FISParser
from numpy import sign

class Controller(object):
	def __init__(self):
		rospy.init_node('controller')
		self.timer = None
		self.state_sub = rospy.Subscriber('/lezl/state',TwistStamped,self.calc_error)
		self.landed_pub = rospy.Publisher('lezl/landed',Bool,queue_size=10)
		self.controller_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped,queue_size=10)
		self.controller_acc_pub = rospy.Publisher('/mavros/setpoint_accel/accel',Vector3Stamped,queue_size=10)
		self.active_sub = rospy.Subscriber('/tracker/tracked',Bool,self.activate)
		self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
		self.srv = Server(pid_gainsConfig,self.get_gains)
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
		self.accel = Vector3Stamped()
		self.rate = rospy.Rate(20)
		self.active = False
		self.landfn = self.land
		self.control = self.calc_effort
		self.control_pub = self.controller_pub
		self.message = self.effort
		self.run()
		rospy.spin()

	def land(self):
		self.arming_client(False)
		self.landed_pub.publish(True)
		sys.exit(0)
		self.landfn = self.null
		rate = rospy.Rate(0.1)
		rate.sleep()
		self.z_s = 10
		rospy.Timer(rospy.Duration(25),self.setpoints,oneshot=True)
		self.run()

	def setpoints(self,timerevent):
		self.z_s=0.25
		self.landfn = self.null
		self.landed_pub.publish(False)

	def null(self):
		pass

	def run(self,timerevent=None):
		self.t = rospy.Time.now()
		if self.timer:
			self.timer.shutdown()
		while not rospy.is_shutdown():
			if (rospy.Time.now() - self.t) > rospy.Duration(2.5):
				self.timer = rospy.Timer(rospy.Duration(2.5),self.run) #will call run() every 5 secs
				return
			elif not self.active:
				self.rate.sleep()
				continue
			self.control()
			self.effort.header.stamp = rospy.Time.now()
			self.control_pub.publish(self.message)
			self.rate.sleep()

	def activate(self,data):
		self.active = data.data

	def calc_error(self,data):
		t = data.header.stamp
		pos = data.twist.linear
		dx, dy, dz = (-(self.x_s - pos.x),-(self.y_s - pos.y),(self.z_s - pos.z))
		if abs(dz) <= self.z_s:
			self.landfn()
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
			if dt == 0:
				return
			self.Ed = (value - self.E)/dt
#			self.Ei += ((value + self.E)/2)*dt
			self.Ei += self.E*dt
			self.E = value
			self.t = t
#			rospy.loginfo('(dt,E,Ei,Ed)=(%0.4f,%0.4f,%0.4f,%0.4f)'%(dt,self.E,self.Ei,self.Ed))

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
		effort.linear.x += self.combine(self.dx,0)
		effort.linear.y += self.combine(self.dy,1)
		effort.linear.z = self.combine(self.dz,2)
		effort.angular.z = self.combine(self.dT,3)
		self.message.twist = effort

	def calc_accel(self):
		accel = Vector3()
		accel.x = self.combine(self.dx,0)
		accel.y = self.combine(self.dy,1)
		accel.z = self.combine(self.dz,2)
		self.message.vector = accel

	def combine(self,error,index):
		u = error.E*Kp[index] + error.Ed*Kd[index] + error.Ei*Ki[index]
		return u if abs(u) <= self.u[index] else u/abs(u)*self.u[index]

	def get_gains(self,config,level):
		xy_p = config['xy_vel_p']
		xy_i = config['xy_vel_i']
		xy_d = config['xy_vel_d']
		self.Kp[0] = xy_p
		self.Kp[1] = xy_p
		self.Ki[0] = xy_i
		self.Ki[1] = xy_i
		self.Kd[0] = xy_d
		self.Kd[1] = xy_d
		return config

class Fuzzy(Controller):
	import subprocess as sp
	def __init__(self,xlims,ylims,zlims,Tlims):
		fispath,_ = self.sp.Popen(['rospack','find', 'irols'],stdout=self.sp.PIPE).communicate()
		fispath = os.path.join(fispath.strip(),'configs','lezl3.fis')
		fisparsers = [FISParser(fispath) for f in range(4)]
		fises = [fp.fis for fp in fisparsers]
		names = ['x','y','z','T']
		for fis,name in zip(fises,names):
			fis.name = name
		self.vx = fises[0]
		self.vy = fises[1]
		self.vz = fises[2]
		self.phi = fises[3]
		self.lims = [xlims,ylims,zlims,Tlims]
		super(Fuzzy,self).__init__()

	def calc_effort(self):
		effort = Twist()
		effort.linear.x = -self.vx.evalfis([self.dx.E/self.lims[0][0],self.dx.Ed/self.lims[0][1]])*self.lims[0][2]
		effort.linear.y = -self.vy.evalfis([self.dy.E/self.lims[1][0],self.dy.Ed/self.lims[1][1]])*self.lims[1][2]
		effort.linear.z = -self.vz.evalfis([self.dz.E/self.lims[2][0],self.dz.Ed/self.lims[2][1]])*self.lims[2][2]
		effort.angular.z = self.phi.evalfis([self.dT.E/self.lims[3][0],self.dT.Ed/self.lims[3][1]])*self.lims[3][2]
		self.message.twist = effort

	def get_gains(self,config,level):
		return config


if __name__ == "__main__":
#	Kp = (0.8,0.8,0.8,10)
	Kp = [5,5,1.2,4]
	Kd = [0.02,0.02,0.01,0.03]
	Ki = [0.002,0.002,0.001,0.003]
#	Kd = [0,0,0,0]
#	Ki = (0,0,0,0)
	u_max = (3.5,3.5,0.25,2)

	vx_lims = [5,5,3]
	vy_lims = [5,5,3]
	vz_lims = [5,5,3]
	phi_lims = [3,8,5]
	try:
#		control = PID(Kp,Kd,Ki,u_max)
		control = Fuzzy(vx_lims,vy_lims,vz_lims,phi_lims)
	except rospy.ROSInterruptException:
		pass
