#!/usr/bin/env python
import rospy
import smach
import smach_ros

from nav_msgs.msg import Odometry

from irols.msg import DoArmGoal, DoArmAction, DoLandAction, DoSeekAction, DoSeekGoal, DoFLCAction

import numpy as np
from numpy.linalg import norm

def covariance_cb(ud, msg):
    cov = msg.pose.covariance
    cov_mx = np.matrix(cov).reshape((6,6))[:3,:3]
    if norm(cov_mx) < 0.1:
        return False
    else:
        return True

def main():
    rospy.init_node('commander')
    
    com = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    
    with com:
        smach.StateMachine.add('ARM',
                               smach_ros.SimpleActionState('arm_action',
                                                           DoArmAction,
                                                           goal=DoArmGoal(arm_cmd=DoArmGoal.ARM)),
                               {'succeeded':'TRACK',
                                'aborted':'aborted'})
        
        track_machine = smach.Concurrence(outcomes=['locked','notready'],
                                          default_outcome='notready',
                                          outcome_map={'locked':
                                              { 'COV_MONITOR':'invalid',
                                                'SEEK':'succeeded'}})
        
        with track_machine:
            
            smach.Concurrence.add('SEEK',smach_ros.SimpleActionState('seek_action',
                                                                       DoSeekAction,
                                                                       goal=DoSeekGoal(alt_sp=5)))

            smach.Concurrence.add('COV_MONITOR',smach_ros.MonitorState('odometry/filtered',
                                                                        Odometry,
                                                                        covariance_cb))

        smach.StateMachine.add('TRACK',track_machine,
                               transitions={'notready':'TRACK',
                                            'locked':'APPROACH'})
        smach.StateMachine.add('APPROACH',smach_ros.SimpleActionState('flc_action',
                                                                      DoFLCAction),
                            {'succeeded':'LAND',
                             'aborted':'TRACK',
                             'preempted':'LAND'})
        smach.StateMachine.add('LAND',smach_ros.SimpleActionState('land_action',
                                                              DoLandAction),
                           {'succeeded':'succeeded',
                            'aborted':'aborted'})

    smach_introspect = smach_ros.IntrospectionServer('commander_viewer',com,'COMMANDER')
    smach_introspect.start()
    outcome = com.execute()
    smach_introspect.stop()
    rospy.loginfo('State Machine completed with status {0}'.format(outcome))

if __name__ == '__main__':
    main()
