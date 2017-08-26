#!/usr/bin/env python
import rospy

import actionlib

from irols.msg import DoLandMachineGoal, DoLandMachineAction

from yapflm import fisyaml

def land_mach_client():
    client = actionlib.SimpleActionClient('land_machine_action',DoLandMachineAction)
    client.wait_for_server()
    fis_names = ['x_fis','y_fis','z_fis','T_fis']
    fis_dicts = [rospy.get_param(name) for name in fis_names]
    fis_list = [fisyaml.fis_from_dict(d) for d in fis_dicts]
    fis_array = [fisyaml.fis_to_ros_msg(f) for f in fis_list]
    goal = DoLandMachineGoal()
    goal.fis_array.fis_array = fis_array
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('test_land_mach_action')
        result = land_mach_client()
        print result
    except rospy.ROSInterruptException:
        pass
