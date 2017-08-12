#!/usr/bin/env python
import rospy
import smach
import smach_ros

from nav_msgs.msg import Odometry

from irols.msg import (DoSeekAction, DoSeekGoal,
                       DoFitnessAction,
                       DoFLCAction,
                       DoLandAction,
                       DoLandMachAction)

import numpy as np
from numpy.linalg import norm

def covariance_cb(ud, msg):
    cov = msg.pose.covariance
    cov_mx = np.matrix(cov).reshape((6,6))[:3,:3]
    if norm(cov_mx) < 0.1:
        return False
    else:
        return True
        

land_machine = smach.StateMachine(outcomes=['succeeded',
                                            'preempted',
                                            'aborted'],
                                  input_keys=['fis_array'],
                                  output_keys=['cost'])

with land_machine:
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
    
    approach_land_machine = smach.StateMachine(outcomes=['succeeded',
                                                'preempted',
                                                'aborted'])
    with approach_land_machine:
        smach.StateMachine.add('APPROACH',smach_ros.SimpleActionState('flc_action',
                                                                      DoFLCAction,
                                                                      goal_slots=['fis_array']),
                            transitions={'succeeded':'LAND',
                             'aborted':'TRACK',
                             'preempted':'LAND'},
                            remapping={'fis_array':'fis_array'})
        smach.StateMachine.add('LAND',smach_ros.SimpleActionState('land_action',
                                                              DoLandAction),
                           {'succeeded':'succeeded',
                            'aborted':'aborted'})
    
    individual_machine = smach.Concurrence(outcomes=['succeeded',
                                                  'aborted'],
                                            default_outcome='aborted',
                                            child_termination_cb=lambda ctcb:True,
                                            outcome_map={'succeeded':{'APPROACH_LAND':'succeeded',
                                                                      'FITNESS':'preempted'}})

    with individual_machine:
        smach.Concurrence.add('APPROACH_LAND',approach_land_machine)
        
        smach.Concurrence.add('FITNESS',smach_ros.SimpleActionState('fitness_action',
                                                                    DoFitnessAction,
                                                                    result_slots=['cost'],
                                                                    remapping={'cost':'cost'}))

    smach.StateMachine.add('INDIVIDUAL_RUN',individual_machine,
                           transitions={'succeeded':'succeeded',
                                        'aborted':'aborted'})

land_machine_action = smach_ros.ActionServerWrapper(
        server_name='land_machine_action',
        action_spec=DoLandMachAction,
        wrapped_container=land_machine,
        succeeded_outcomes=['succeeded'],
        preempted_outcomes=['preempted'],
        aborted_outcomes=['aborted'],
        goal_key='fis_array',
        result_key='cost')

smach_introspect = smach_ros.IntrospectionServer('individual_viewer',land_machine,'INDIVIDUAL')
smach_introspect.start()
land_machine_action.run_server()
rospy.loginfo('land_machine_action: online')

rospy.spin()
