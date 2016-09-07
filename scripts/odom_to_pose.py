import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class Remapper(object):
	def __init__(self):
		self.publisher = rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,
					queue_size=10)
		self.pose = PoseStamped()
		rospy.init_node('pioneer_publisher')
		self.subscriber = rospy.Subscriber('/p3at/odom',Odometry,self.translate)

	def translate(self, msg):
		self.pose.pose = msgs.pose.pose

	def publish(self)
