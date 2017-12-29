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
#    rosbag_file = '/home/stocktno/catkin_ws/src/irols/bags/static_expert1.bag'
    rosbag_file = '/home/stocktno/catkin_ws/src/irols/bags/ind_test.bag'

bag = rosbag.Bag(rosbag_file)

simstates = [(t, msg) for topic, msg, t in bag.read_messages(topics=['/irols_sim/state'])]
#smach_states = [(t, msg) for topic, msg, t in bag.read_messages(topics=['/commander_viewer/smach/container_status'])]

start = simstates[0][0]
end = simstates[1][0]
#start = [x[0] for x in smach_states if x[1].active_states[0] == 'WP2'][0]
#end = [x[0] for x in smach_states if x[1].active_states[0] == 'LAND'][-1]
#start = rospy.Time(0)
#end = rospy.Time(9999999999999)

gz_msgs = [(t, msg) for topic, msg, t in bag.read_messages(topics=['/gazebo/model_states'],
                                                                start_time=start,
                                                                end_time=end)]
imu_msgs = [(t, msg) for topic, msg, t in bag.read_messages(topics=['/mavros/imu/data'],
                                                                start_time=start,
                                                                end_time=end)]
odom_msgs = [(t, msg) for topic, msg, t in bag.read_messages(topics=['/odometry/filtered'],
                                                                start_time=start,
                                                                end_time=end)]

ix_lezl = gz_msgs[0][1].name.index('lezl')
ix_p3at = gz_msgs[0][1].name.index('pioneer3at_lander')

gz_tm = np.array([t.to_sec() for t,m in gz_msgs])
imu_tm = np.array([t.to_sec() for t,m in imu_msgs])
odom_tm = np.array([t.to_sec() for t,m in odom_msgs])

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

odom_vel_hist = []
for odom_msg in odom_msgs:
    twist = odom_msg[1].twist.twist.linear
    odom_vel_hist.append((twist.x, twist.y, twist.z))
odom_vel_hist = np.array(odom_vel_hist)
odom_vel_norm = np.divide(odom_vel_hist.T,np.linalg.norm(odom_vel_hist,axis=1).T).T

odom_norm_interp = np.array([np.interp(gz_tm,odom_tm,x) for x in odom_vel_norm.T]).T
vel_quiver = lezl_pos_hist + odom_norm_interp

tm_mx = np.intersect1d(gz_tm, imu_tm)
ix_mx_gz = np.searchsorted(gz_tm, tm_mx)
ix_mx_imu = np.searchsorted(imu_tm, tm_mx)

#imu_accel_hist /= (2*G)

lezl_pos_hist = np.array(lezl_pos_hist)
#lezl_pos_hist[:,2] = lezl_pos_hist[0, 2]
p3at_pos_hist = np.array(p3at_pos_hist)
p3at_pos_hist[:,2] += 0.5 # raise the goal point to Lezl's C.G.


#plt.ion()
fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')
#ax.plot(lezl_pos_hist[:,0], lezl_pos_hist[:,1], lezl_pos_hist[:,2])
#
#ax.quiver(lezl_pos_hist[ix_mx_gz,0], lezl_pos_hist[ix_mx_gz,1], lezl_pos_hist[ix_mx_gz,2],
##ax.quiver(lezl_pos_hist[::50,0], lezl_pos_hist[::50,1], lezl_pos_hist[::50,2],
##          odom_norm_interp[::50,0], odom_norm_interp[::50,1], odom_norm_interp[::50,2],
#          imu_accel_hist[ix_mx_imu,0], imu_accel_hist[ix_mx_imu,1], imu_accel_hist[ix_mx_imu,2],
#          pivot= 'tail', arrow_length_ratio = 0.1, length=0.1)
#J = 
#plt.ioff()

pos_error = np.abs(lezl_pos_hist - p3at_pos_hist)
xy_dist = np.linalg.norm(pos_error[:,:2], axis=1)
slant_range = np.linalg.norm(pos_error, axis=1)
ix = slant_range > 0.1
A = np.vstack([gz_tm[ix] - gz_tm[0],np.ones(len(gz_tm[ix]))]).T
slope, yint = np.linalg.lstsq(A, slant_range[ix])[0]
slope = -abs(slope)
log_inv_slant_range = np.log(1/slant_range)
err_cost = xy_dist**2 * (log_inv_slant_range  - log_inv_slant_range.min())
tm_cost = (slant_range - yint - slope*(gz_tm-gz_tm[0]))**2
final_cost = np.linalg.norm(pos_error[-1])
cost = err_cost + tm_cost

xy_ax = fig.add_subplot(511)
slant_ax = fig.add_subplot(512)
xy_cost_ax = fig.add_subplot(513)
tm_cost_ax = fig.add_subplot(514)
cost_ax = fig.add_subplot(515)

xy_ax.plot(gz_tm[ix],xy_dist[ix])
slant_ax.plot(gz_tm[ix],slant_range[ix])
xy_cost_ax.plot(gz_tm[ix],err_cost[ix])
tm_cost_ax.plot(gz_tm[ix],tm_cost[ix])
cost_ax.plot(gz_tm[ix],cost[ix])

print('Cost is {}'.format(np.sum(cost[ix]**2) + final_cost**2))
