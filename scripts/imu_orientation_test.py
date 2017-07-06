import rospy

from sensor_msgs.msg import Imu

from tf.transformations import euler_from_quaternion as efq

def imu_cb(data):
    o = data.orientation
    q = [o.x,o.y,o.z,o.w]
    r,p,y = efq(q)
    print('rpy',r,p,y)

rospy.init_node('imu_test')

imu_sub = rospy.Subscriber('mavros/imu/data',Imu,imu_cb)

while not rospy.is_shutdown():
    rospy.spin()
