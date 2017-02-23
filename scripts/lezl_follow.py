#!/usr/bin/env python
import rospy
import tf
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Bool
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from apriltags_ros.msg import AprilTagDetectionArray

class LEZL:
	def __init__(self):
		rospy.init_node('lezl')
		self.pose = PoseStamped()
		self.state_sub = rospy.Subscriber('/mavros/state',State,self.state_cb)
		self.odom_sub = rospy.Subscriber('/p3at/odom',Odometry,self.odom_cb)
		self.tracked_sub = rospy.Subscriber('/tracker/tracked',Bool,self.track_cb)
		self.landed_sub = rospy.Subscriber('/lezl/landed',Bool,self.landed_cb)
		self.track_pub = rospy.Publisher('/tracker/tracked',Bool,queue_size=10)
		self.tag_track_pub = rospy.Publisher('/tag_detections/tracked',Bool,queue_size=10)
		self.state_pub = rospy.Publisher('/lezl/state',TwistStamped,queue_size=10)
		self.heading_sub = rospy.Subscriber('/tracker/heading',TwistStamped,self.head_cb)
		self.apriltag_sub = rospy.Subscriber('/tag_detections',AprilTagDetectionArray,self.parse_tags)
		local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,queue_size=10)
		self.cam_info_sub = rospy.Subscriber('/camera1/camera_info',CameraInfo,self.get_cam_info)
		self.cam_info_pub = rospy.Publisher('/landing_pad/camera_info',CameraInfo,queue_size=10)
		local_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped,queue_size=10)
		self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
		self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode',SetMode)
		self.state = State()
		self.tag_track = Bool()
		self.cam_info = CameraInfo()
		self.tracked = False
		self.landed = False
		self.lezl_state = TwistStamped()
		rate = rospy.Rate(20)
		while self.state.connected:
			rospy.spinOnce()
			rate.sleep()

		self.pose.pose.position.x = 0
		self.pose.pose.position.y = 0
		self.pose.pose.position.z = 25

		wait = rospy.Time.now()
		offb = True
		print "Starting Mission"
		while not rospy.is_shutdown():
			if self.landed:
				print "Landed!!!"
				break
#			print (rospy.Time.now() - wait)>rospy.Duration(5)
			if (self.state.mode != "OFFBOARD" and 
				(rospy.Time.now() - wait) > rospy.Duration(5)):
				print "Attempting to enter OFFBOARD"
				self.mode_switch("OFFBOARD")
				start = rospy.Time.now()
			else:
				if (not self.state.armed and
					(rospy.Time.now() - wait) > rospy.Duration(5)):
					print "Attempting to arm"
					self.arm_cmd(1)
			if not offb and self.state.mode == 'OFFBOARD':
				print "Watching for landing"
				self.watchdog(start)
			self.pose.header.stamp = rospy.Time.now()
			if not self.tracked:
				local_pos_pub.publish(self.pose)
			else:
				pass # Controller() class should be handling this
				#local_vel_pub.publish(self.heading)
			rate.sleep()
			
	def track_cb(self,data):
		self.tracked = data.data

	def landed_cb(self,data):
		self.landed = data.data

	def get_cam_info(self,data):
		self.cam_info = data
		self.cam_info_sub.unregister()

	def parse_tags(self,data):
		if len(data.detections) > 0:
			self.tag_track.data = True
			data = data.detections[0].pose
			dx = data.pose.position.x
			dy = data.pose.position.y
			dz = data.pose.position.z
			qx = data.pose.orientation.x
			qy = data.pose.orientation.y
			qz = data.pose.orientation.z
			qw = data.pose.orientation.w
			r,p,y = tf.transformations.euler_from_quaternion((qx,qy,qz,qw))
			self.lezl_state.header.stamp = rospy.Time.now()
			self.lezl_state.twist.linear.x = dx
			self.lezl_state.twist.linear.y = dy
			self.lezl_state.twist.linear.z = dz
			self.lezl_state.twist.angular.z = y
			self.state_pub.publish(self.lezl_state)
		else:
			self.tag_track.data = False
		self.tag_track_pub.publish(self.tag_track)
	
	def head_cb(self,data):
		self.heading = data
		self.heading.twist.linear.x *= 0.25
		self.heading.twist.linear.y *= 0.25
		self.heading.twist.linear.z = -0.2 if data.twist.linear.z>1.5 else 0.2
#		self.heading.twist.linear.z = 0.1*(2 - data.twist.linear.z)

	def state_cb(self,data):
		self.state = data

	def odom_cb(self,data):
		x = data.pose.pose.position.x
		y = data.pose.pose.position.y
		self.pose.pose.position.x = x
		self.pose.pose.position.y = y
		self.pose.pose.position.z = 25
		ori = (data.pose.pose.orientation.x,
			data.pose.pose.orientation.y,
			data.pose.pose.orientation.z,
			data.pose.pose.orientation.w)
		br = tf.TransformBroadcaster()
		br.sendTransform((x,y,0),
				ori,
				rospy.Time.now(),
				'rover',
				'world')
		br.sendTransform((0,0,0.33),
				(0,0,0,1),
				rospy.Time.now(),
				'platform',
				'rover')
		

	def watchdog(self,start):
		rate = rospy.Rate(1)
		while not rospy.is_shutdown():
			now = rospy.Time.now()
			if now-start > rospy.Duration(30):
				self.pose.pose.position.x = 2
				self.pose.pose.position.y = 2
				self.pose.pose.position.z = 0
				break
		self.land_time = rospy.Time.now()
		rospy.Subscriber('/mavros/imu/data',Imu,self.imu_cb)

	def imu_cb(self,data):
		threshold = 0.2
		while not rospy.is_shutdown():
			if (abs(data.linear_acceleration.z - 9.81) < threshold and
				(rospy.Time.now() - self.land_time) > rospy.Duration(5)):
				self.arm_cmd(0)			

	def arm_cmd(self,arming):
		if arming:
			self.arming_client(True)
		else:
			self.arming_client(False)

	def mode_switch(self,newmode):
		self.set_mode_client(custom_mode=newmode)

if __name__ == '__main__':
	lezl = LEZL()
