"""
Script to test out different fitness funcitons on known good data to try and
figure out what parameters need to be tuned.
"""
#!/usr/bin/env python
import rospy
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

G = 9.81

if len(sys.argv) > 1:
    rosbag_file = sys.argv[1]
else:
    rosbag_file = '/home/stocktno/catkin_ws/src/irols/bags/static_expert2.bag'

bag = rosbag.Bag(rosbag_file)

simstates = [(t, msg) for topic, msg, t in bag.read_messages(topics=['/irols_sim/state'])]
#smach_states = [(t, msg) for topic, msg, t in bag.read_messages(topics=['/commander_viewer/smach/container_status'])]

#start = simstates[0][0]
#end = simstates[1][0]
#start = [x[0] for x in smach_states if x[1].active_states[0] == 'WP2'][0]
#end = [x[0] for x in smach_states if x[1].active_states[0] == 'LAND'][-1]
start = rospy.Time(0)
end = rospy.Time(9999999999999)

gz_msgs = [(t, msg) for topic, msg, t in bag.read_messages(topics=['/gazebo/model_states'],
                                                                start_time=start,
                                                                end_time=end)]
imu_msgs = [(t, msg) for topic, msg, t in bag.read_messages(topics=['/mavros/imu/data'],
                                                                start_time=start,
                                                                end_time=end)]

ix_lezl = gz_msgs[0][1].name.index('lezl')
ix_p3at = gz_msgs[0][1].name.index('pioneer3at_lander')

gz_tm = np.array([t.to_sec() for t,m in gz_msgs])
imu_tm = np.array([t.to_sec() for t,m in imu_msgs])

imu_accel_hist = []
for imu_msg in imu_msgs:
    accels = imu_msg[1].linear_acceleration
    imu_accel_hist.append((accels.x, accels.y, accels.z))
imu_accel_hist = np.array(imu_accel_hist)

lezl_pos_hist, p3at_pos_hist = [], []
for gz_msg in gz_msgs:
    lezl_pos = gz_msg[1].pose[ix_lezl].position
    p3at_pos = gz_msg[1].pose[ix_p3at].position
    
    lezl_pos_hist.append((lezl_pos.x, lezl_pos.y, lezl_pos.z))
    p3at_pos_hist.append((p3at_pos.x, p3at_pos.y, p3at_pos.z))

tm_mx = np.intersect1d(gz_tm, imu_tm)
ix_mx_gz = np.searchsorted(gz_tm, tm_mx)
ix_mx_imu = np.searchsorted(imu_tm, tm_mx)

#imu_accel_hist /= (2*G)

lezl_pos_hist = np.array(lezl_pos_hist)
p3at_pos_hist = np.array(p3at_pos_hist)

#fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')
#ax.plot(lezl_pos_hist[:,0], lezl_pos_hist[:,1], lezl_pos_hist[:,2])

#ax.quiver(lezl_pos_hist[ix_mx_gz,0], lezl_pos_hist[ix_mx_gz,1], lezl_pos_hist[ix_mx_gz,2],
#          imu_accel_hist[ix_mx_imu,0], imu_accel_hist[ix_mx_imu,1], imu_accel_hist[ix_mx_imu,2],
#          pivot= 'tail', arrow_length_ratio = 0.1)
#J = 

