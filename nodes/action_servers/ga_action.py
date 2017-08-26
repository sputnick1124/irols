#!/usr/bin/env python
import rospy
import actionlib

import irols.msg
from yapflm import ga, fisyaml, cascade, gfs

def land_machine_action_client(cascade):
    """
    assume that we are only giving it an xy fIS and a z FIS for now.
    """
    rospy.loginfo('land_machine_action_client: received individual')
    client = actionlib.SimpleActionClient('land_machine_action',irols.msg.DoLandMachineAction)
    client.wait_for_server()
    rospy.loginfo('land_machine_action_client: client is live')
    fis_array = irols.msg.FISArray(fis_array=[fisyaml.fis_to_ros_msg(fis) for fis in cascade.gfs])
    fis_array = [fis_array.fis_array[0]]*2 + [fis_array.fis_array[1]]*2
    goal = irols.msg.DoLandMachineGoal()
    goal.fis_array.fis_array = fis_array
    client.send_goal(goal)
    rospy.loginfo('land_machine_action_client: goal is sent')
    client.wait_for_result()
    result = client.get_result()
    return result.fitness

class GAServer(object):
    _feedback = irols.msg.DoGAFeedback()
    _result = irols.msg.DoGAResult()
    
    def __init__(self,name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                irols.msg.DoGAAction,
                                                execute_cb=self.execute,
                                                auto_start=False)

        self.ga = ga.GA(popSize=10,genMax=5,cpus=1)
        xy_fis_dict = rospy.get_param('x_fis')
        xy_fis = fisyaml.fis_from_dict(xy_fis_dict)
        z_fis_dict = rospy.get_param('z_fis')
        z_fis = fisyaml.fis_from_dict(z_fis_dict)

        gfs_list = [gfs.GFS(fis=fis) for fis in [xy_fis,z_fis]]

        csc = cascade.GenFuzzyCascade(*gfs_list)
        
        self.ga.add_prototype(csc)
        self.ga.init_population()
        self.ga.seed_pop(csc)
        self.ga.add_fitness(land_machine_action_client)
        
        self._as.start()
        rospy.loginfo('{}: online'.format(self._action_name))

    def execute(self,goal):
        rospy.loginfo('{0}: received goal: {1}'.format(self._action_name,goal))
        self.ga.genMax = goal.num_gen
        for gen in xrange(goal.num_gen):
            self._feedback.curr_gen = gen
            self.ga.eval_population()
            self._feedback.curr_min_cost = self.ga.fitness_hist[-1]
            self.ga.iter_generation()
        self.eval_population()
        self._result.min_cost = self.fitness_hist[-1]
        fis_array = irols.msg.FISArray(fis_array=[fisyaml.fis_to_ros_msg(fis) for fis in cascade])
        fis_array = [fis_array[0]]*2 + [fis_array[1]]*2
        self._result.fis_array = fis_array
        self._as.set_succeeded(self.__result)

if __name__ == '__main__':
    rospy.init_node('ga_action_server')
    server = GAServer('ga_action')
    rospy.spin()