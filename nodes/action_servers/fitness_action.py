#!/usr/bin/env python
import rospy
import actionlib

import irols.msg
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates

from math import sqrt
from itertools import izip

def unpack_xyz(msg):
    return (msg.x, msg.y, msg.z)

class FitnessServer(object):
    _feedback = irols.msg.DoFitnessFeedback
    _result = irols.msg.DoFitnessResult

    def __init__(self,name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                irols.msg.DoFitnessAction,
                                                execute_cb=self.execute,
                                                auto_start=False)

        self.accels = []
        rospy.wait_for_message('mavros/imu/data',Imu)
        self.imu_sub = rospy.Subscriber('mavros/imu/data',
                                        Imu,
                                        self.handle_imu)
        
        self.state = irols.msg.SimState()
        self.sim_state_pub = rospy.Publisher('irols_sim/state',
                                             irols.msgs.SimState,
                                             queue_size=2,
                                             latch=True)
 
        self._as.start() 
        rospy.loginfo('{}: online'.format(self._action_name))


    def execute(self):
        self.state.state = irols.msg.SimState().ACTIVE
        self.state.header.stamp = rospy.Time.now()
        self.sim_state_pub.publish(self.state)
        rate = rospy.Rate(10)
        while not self._as.is_preempted():
            rate.sleep()
        
        
        self.state.state = irols.msg.SimState().INACTIVE
        self.state.header.stamp = rospy.Time.now()
        self.sim_state_pub.publish(self.state)
        
        gz_poses = rospy.wait_for_message('gazebo/model_states',ModelStates)
        p3at_idx = gz_poses.name.index('pioneer_lander')
        lezl_idx = gz_poses.name.index('lezl_cam')
        x_offset = gz_poses.pose[p3at_idx].position.x-gz_poses.pose[lezl_idx].position.x
        y_offset = gz_poses.pose[p3at_idx].position.y-gz_poses.pose[lezl_idx].position.y
        pos_err = sqrt(x_offset**2 + y_offset**2)
        err_tups = (unpack_xyz(msg) for msg in self.accels)
        xddots,yddots,zddots = izip(*err_tups)
        x_rms = sqrt(sum(x**2 for x in xddots)/len(self.accels))
        y_rms = sqrt(sum(y**2 for y in yddots)/len(self.accels))
#        z_rms = sqrt(sum(z**2 for z in zddots)/len(self.accels))
        self._result.cost = 2*pos_err + 0.5*(x_rms+y_rms)
        self._as.set_preempted(self._result)

    def handle_imu(self,data):
        if self._as.is_active():
            self.accels.append(data.linear_acceleration)
            xddot = self.accels[-1].x
            yddot = self.accels[-1].y
            self._feedback.rms_h_accel = sqrt(sum(xddot**2 + yddot**2)/2)
            self._as.publish_feedback(self._feedback)
