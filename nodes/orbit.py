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

alt = 3.5

waypoints['WP1'].goal_pos.position.x = 3.25
waypoints['WP1'].goal_pos.position.y = 3.25
waypoints['WP1'].goal_pos.position.z = alt

waypoints['WP2'].goal_pos.position.x = 2.25
waypoints['WP2'].goal_pos.position.y = 2.25
waypoints['WP2'].goal_pos.position.z = alt

waypoints['WP3'].goal_pos.position.x = 2.25
waypoints['WP3'].goal_pos.position.y = 3.25
waypoints['WP3'].goal_pos.position.z = alt

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
                                                                 goal=waypoints['WP2']))
            smach.Sequence.add('WP5',smach_ros.SimpleActionState('wp_action',
                                                                 DoWPAction,
                                                                 goal=waypoints['WP3']))
            smach.Sequence.add('WP6',smach_ros.SimpleActionState('wp_action',
                                                                 DoWPAction,
                                                                 goal=waypoints['WP1']))
            smach.Sequence.add('WP7',smach_ros.SimpleActionState('wp_action',
                                                                 DoWPAction,
                                                                 goal=waypoints['WP3']))
            smach.Sequence.add('WP8',smach_ros.SimpleActionState('wp_action',
                                                                 DoWPAction,
                                                                 goal=waypoints['WP2']))
            smach.Sequence.add('WP9',smach_ros.SimpleActionState('wp_action',
                                                                 DoWPAction,
                                                                 goal=waypoints['WP1']))
         
        smach.StateMachine.add('WAYPOINTS',waypoint_sequence,
                                transitions={'succeeded':'LAND',
                                             'preempted':'preempted',
                                             'aborted':'aborted'})
        
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