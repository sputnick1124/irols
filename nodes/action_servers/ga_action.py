#!/usr/bin/env python
import rospy
import actionlib

import irols.msg
from yapflm import ga, fisyaml, cascade, gfs

from operator import itemgetter

def land_machine_action_client(csc):
    """
    assume that we are only giving it an xy fIS and a z FIS for now.
    """
    rospy.loginfo('land_machine_action_client: received individual')
    client = actionlib.SimpleActionClient('land_machine_action',irols.msg.DoLandMachineAction)
    client.wait_for_server()
    rospy.loginfo('land_machine_action_client: client is live')
    fis_array = [fisyaml.fis_to_ros_msg(fis) for fis in csc.gfs]
    fis_array = [fis_array[0]]*2 + [fis_array[1]]*2
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

        self.ga = ga.GA(popSize=50,genMax=5,cpus=1)
        xy_fis_dict = rospy.get_param('x_fis')
        xy_fis = fisyaml.fis_from_dict(xy_fis_dict)
        z_fis_dict = rospy.get_param('z_fis')
        z_fis = fisyaml.fis_from_dict(z_fis_dict)

        gfs_list = [gfs.GFS(fis=fis,in_ranges=[(-1,1),(-1,1)],out_ranges=[(-1,1)]) for fis in [xy_fis,z_fis]]

        csc = cascade.GenFuzzyCascade(*gfs_list)
        
        self.ga.add_prototype(csc)
        self.ga.init_population()
        self.ga.seed_pop(csc)
        self.ga.add_fitness(land_machine_action_client)
        self.cur_best_ind = 0
        
        self._as.start()
        rospy.loginfo('{}: online'.format(self._action_name))

    def execute(self,goal):
        rospy.loginfo('{0}: received goal: {1}'.format(self._action_name,goal))
        self.ga.genMax = goal.num_gen
        self.ga.popSize = goal.pop_size
        self._feedback.curr_min_cost = float('inf')
        for gen in xrange(goal.num_gen-1):
            self._feedback.curr_gen = gen
            self._as.publish_feedback(self._feedback)
            self.eval_population()
            self.ga.iter_generation()
        self._feedback.curr_gen += 1
        self.eval_population()
        self._result.min_cost = self.ga.fitness_hist[-1]
        fis_array = [fisyaml.fis_to_ros_msg(fis) for fis in self.ga.pop_curr[0].gfs]
        fis_array = [fis_array[0]]*2 + [fis_array[1]]*2
        self._result.fis_array.fis_array = fis_array
        self._as.set_succeeded(self._result)
    
    def eval_population(self):
        """
        Reach into GA and do things manually for the sake of reporting
        """
        fitnessvalues = []
        for i,ind in enumerate(self.ga.pop_curr):
            if self._as.is_preempt_requested():
                self._result.min_cost = self._feedback.curr_min_cost
                fis_array = [fisyaml.fis_to_ros_msg(fis) for fis in self.ga.pop_curr[self.cur_best_ind].gfs]
                fis_array = [fis_array[0]]*2 + [fis_array[1]]*2
                self._result.fis_array.fis_array = fis_array
            fitnessvalues.append((ind,self.ga.fitness(ind)))
            if fitnessvalues[-1][-1]<self._feedback.curr_min_cost:
                self._feedback.curr_min_cost = fitnessvalues[-1][-1]
                self.cur_best_ind = i
            self._feedback.curr_cost = fitnessvalues[-1][-1]
            self._feedback.curr_ind = i
            self._as.publish_feedback(self._feedback)
        sorted_fitness = zip(*sorted(fitnessvalues,key=itemgetter(1)))
        self.ga.pop_curr = sorted_fitness[0]
        self.ga.fitness_hist.append(sorted_fitness[1][0])

if __name__ == '__main__':
    rospy.init_node('ga_action_server')
    server = GAServer('ga_action')
    rospy.spin()