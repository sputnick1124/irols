#!/usr/bin/env python
import rospy
import smach
import smach_ros

from nav_msgs.msg import Odometry

from irols.msg import DoArmGoal, DoArmAction, DoWPGoal, DoWPAction, DoLandAction, DoTrackAction, DoTrackGoal

import numpy as np
from numpy.linalg import norm

waypoints = {name:DoWPGoal() for name in ['WP0','WP1','WP2','WP3','LAND']}

waypoints['WP0'].goal_pos.position.z = 2

waypoints['WP1'].goal_pos.position.x = 2
waypoints['WP1'].goal_pos.position.y = 2
waypoints['WP1'].goal_pos.position.z = 2

waypoints['WP2'].goal_pos.position.x = 2
waypoints['WP2'].goal_pos.position.y = -2
waypoints['WP2'].goal_pos.position.z = 2

waypoints['WP3'].goal_pos.position.x = -2
waypoints['WP3'].goal_pos.position.y = -2
waypoints['WP3'].goal_pos.position.z = 2

def covariance_cb(ud, msg):
    cov = msg.pose.covariance
    cov_mx = np.matrix(cov).reshape((6,6))
    if norm(cov_mx) < 1:
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
                               {'succeeded':'WAYPOINTS',
                                'aborted':'aborted'})
        
        waypoint_sequence = smach.Sequence(
                                outcomes=['succeeded','aborted','preempted'],
                                connector_outcome='succeeded')
        
        with waypoint_sequence:
            smach.Sequence.add('TAKEOFF',smach_ros.SimpleActionState('wp_action',
                                                                 DoWPAction,
                                                                 goal=waypoints['WP0']))
            smach.Sequence.add('WP1',smach_ros.SimpleActionState('wp_action',
                                                                 DoWPAction,
                                                                 goal=waypoints['WP1']))
            smach.Sequence.add('WP2',smach_ros.SimpleActionState('wp_action',
                                                                 DoWPAction,
                                                                 goal=waypoints['WP2']))
            smach.Sequence.add('WP3',smach_ros.SimpleActionState('wp_action',
                                                                 DoWPAction,
                                                                 goal=waypoints['WP3']))
            smach.Sequence.add('WP4',smach_ros.SimpleActionState('wp_action',
                                                                 DoWPAction,
                                                                 goal=waypoints['WP0']))
         
        smach.StateMachine.add('WAYPOINTS',waypoint_sequence,
                                transitions={'succeeded':'TRACK',
                                             'preempted':'preempted',
                                             'aborted':'aborted'})

        track_machine = smach.Concurrence(outcomes=['locked','notready'],
                                          default_outcome='notready',
                                          outcome_map={'locked':
                                              { 'COV_MONITOR':'invalid',
                                                'SEEK':'succeeded'}})
        
        with track_machine:
            
            smach.Concurrence.add('SEEK',smach_ros.SimpleActionState('track_action',
                                                                       DoTrackAction,
                                                                       goal=DoTrackGoal(alt_sp=15)))

            smach.Concurrence.add('COV_MONITOR',smach_ros.MonitorState('odometry/filtered',
                                                                        Odometry,
                                                                        covariance_cb))

        smach.StateMachine.add('TRACK',track_machine,
                               transitions={'notready':'TRACK',
                                            'locked':'LAND'})
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