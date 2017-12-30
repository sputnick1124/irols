#!/usr/bin/env python
import rospy
import actionlib
from rospkg import RosPack

import irols.msg
from irols.utils import fitness_fcn
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty

from math import sqrt, log
import numpy as np
from functools import partial
from datetime import datetime
import os

def unpack_xyz(msg):
    return (msg.x, msg.y, msg.z)

class FitnessServer(object):
    _feedback = irols.msg.DoFitnessFeedback()
    _result = irols.msg.DoFitnessResult()

    def __init__(self,name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                irols.msg.DoFitnessAction,
                                                execute_cb=self.execute,
                                                auto_start=False)

        self.time_hist = []
        self.lezl_pos_hist = []
        self.p3at_pos_hist = []
        gz_poses = rospy.wait_for_message('gazebo/model_states',ModelStates)
        while 'lezl' not in gz_poses.name:
            gz_poses = rospy.wait_for_message('gazebo/model_states',ModelStates)
            
        ix_p3at = gz_poses.name.index('pioneer3at_lander')
        ix_lezl = gz_poses.name.index('lezl')
        self.imu_sub = rospy.Subscriber('gazebo/model_states',
                                        ModelStates,
                                        partial(self.handle_gz_states,
                                                ix_lezl=ix_lezl,
                                                ix_p3at=ix_p3at))
        
        self.accels = []
        rospy.wait_for_message('mavros/imu/data',Imu)
        self.imu_sub = rospy.Subscriber('mavros/imu/data',
                                        Imu,
                                        self.handle_imu)
        
        self.state = irols.msg.SimState()
        self.sim_state_pub = rospy.Publisher('irols_sim/state',
                                             irols.msg.SimState,
                                             queue_size=2,
                                             latch=True)
        
        self.reset_service = rospy.ServiceProxy('/gazebo/reset_world',Empty)
        
        rp = RosPack()
        self.log_path = os.path.join(rp.get_path('irols'),'bags','logs')
 
        self._as.start()
        rospy.loginfo('{}: online'.format(self._action_name))


    def execute(self,goal):
        # reset histories (maybe not necessary?)
        rospy.loginfo("{0}: recieved goal: {1}".format(self._action_name, goal))
        self.time_hist = []
        self.lezl_pos_hist = []
        self.p3at_pos_hist = []
        start = rospy.Time.now()
        self.state.state = irols.msg.SimState().ACTIVE
        self.state.header.stamp = start
        self.sim_state_pub.publish(self.state)
        rate = rospy.Rate(10)
        while not self._as.is_preempt_requested():
            if self.accels and (self.accels[-1].z < -9.0): # we're upside down
                rospy.logwarn('Flip detected. Resetting.')
                self.reset_service()
                rospy.sleep(1)
                self.reset_service()
                rospy.sleep(1)
                self.reset_service()
            if (rospy.Time.now() - start) > rospy.Duration(30):
                break
            rate.sleep()
        
        
        self.state.state = irols.msg.SimState().INACTIVE
        self.state.header.stamp = rospy.Time.now()
        self.sim_state_pub.publish(self.state)
        
        gz_tm = np.array(self.time_hist)
        lezl_pos_hist = np.array(self.lezl_pos_hist)
        p3at_pos_hist = np.array(self.p3at_pos_hist)
        p3at_pos_hist[:,2] += 0.537 # raise the goal point to Lezl's C.G.

        pos_error = np.abs(lezl_pos_hist - p3at_pos_hist)
        #xy_dist = np.linalg.norm(pos_error[:,:2], axis=1)
        #slant_range = np.linalg.norm(pos_error, axis=1)
        #ix = slant_range > 0.1
        #A = np.vstack([gz_tm[ix] - gz_tm[0],np.ones(len(gz_tm[ix]))]).T
        #slope, yint = np.linalg.lstsq(A, slant_range[ix])[0]
        #slope = -abs(slope)
        #log_inv_slant_range = np.log(1/slant_range)
        #err_cost = (xy_dist**2)*(log_inv_slant_range  - log_inv_slant_range.min())
        #tm_cost = (slant_range - yint - slope*(gz_tm-gz_tm[0]))**2
        #cost = err_cost + tm_cost
        cost = fitness_fcn(pos_error)[0]
        postfix = datetime.strftime(datetime.now(),"_%y%m%d%H%M%S")
        fname = ''.join(['simdata',postfix,'.txt'])
        fpath = os.path.join(self.log_path, fname)
        np.savetxt(fpath, np.hstack((lezl_pos_hist,p3at_pos_hist)))

        self._result.cost.fitness = cost
        self._as.set_preempted(self._result)
        print("!!!!!!!!!!!!!!!!!!!DONE!!!!!!!!!!!!!!!!!!!!!!")
        

    def handle_imu(self,data):
        if self._as.is_active():
            self.accels.append(data.linear_acceleration)
            xddot = self.accels[-1].x
            yddot = self.accels[-1].y
            zddot = self.accels[-1].z - 9.81
            self._feedback.pos_err = sqrt((xddot**2 + yddot**2 + zddot**2)/3)
            self._as.publish_feedback(self._feedback)

    def handle_gz_states(self,msg,ix_lezl,ix_p3at):
        if self._as.is_active():
            self.time_hist.append(rospy.Time.now().to_sec())
            self.lezl_pos_hist.append(unpack_xyz(msg.pose[ix_lezl].position))
            self.p3at_pos_hist.append(unpack_xyz(msg.pose[ix_p3at].position))
            lx0, ly0, lz0 = self.lezl_pos_hist[0]
            px0, py0, pz0 = self.p3at_pos_hist[0]
            lx, ly, lz = self.lezl_pos_hist[-1]
            px, py, pz = self.p3at_pos_hist[-1]
            xy_dist = sqrt((lx-px)**2 + (ly - py)**2)
            dist = sqrt((lx-px)**2 + (ly - py)**2 + (lz - pz)**2)
            dist0 = sqrt((lx0 - px0)**2 + (ly0 - py0)**2 + (lz0 - pz0)**2)
            self._feedback.pos_err += (xy_dist**2)*(log(1/dist) - log(1/dist0))
            self._as.publish_feedback(self._feedback)

if __name__ == '__main__':
    rospy.init_node('fitness_action_server')
    server = FitnessServer('fitness_action')
    rospy.spin()
