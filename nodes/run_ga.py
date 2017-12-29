#!/usr/bin/env python
import sys
import rospy
import actionlib

from irols.msg import DoGAGoal, DoGAAction #, DoGAFeeback, DoGAResult
from yapflm import fisyaml

def fb_cb(fb):
    rospy.loginfo('gen: {0}, ind: {2}, cur_fitness: {3}, best_fitness: {1}'.format(fb.curr_gen,fb.curr_min_cost,fb.curr_ind,fb.curr_cost))

if __name__ == '__main__':
    rospy.init_node('run_ga')
    client = actionlib.SimpleActionClient('ga_action',DoGAAction)
    client.wait_for_server()
    if len(sys.argv) > 1:
        num_gen = int(sys.argv[1])
        pop_size = int(sys.argv[2])
        rospy.loginfo('Running GA with {0} generations and {1} individuals'.format(num_gen,pop_size))
        goal = DoGAGoal(num_gen=num_gen,
                        pop_size=pop_size)
    else:
        rospy.loginfo('Running GA with {0} generations and {1} individuals'.format(5,20))
        goal = DoGAGoal()
    try:
        client.send_goal(goal,feedback_cb=fb_cb)
        client.wait_for_result()
        result = client.get_result()
        fis_names = ['x_fis','y_fis','z_fis','T_fis']
        for rosfis,name in zip(result.fis_array.fis_array,fis_names):
            fis = fisyaml.fis_from_ros_msg(rosfis)
            fisyaml.fis_to_yaml(fis,name+'result.yaml')
    except rospy.ROSInterruptException():
        client.cancel_all_goals()
    finally:
        client.cancel_all_goals()
