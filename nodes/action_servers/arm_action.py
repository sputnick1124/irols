#!/usr/bin/env python
import rospy
import actionlib

import irols.msg

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolResponse


class ArmServer(object):
    _feedback = irols.msg.DoArmFeedback()
    _result = irols.msg.DoArmResult()

    def __init__(self,name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                irols.msg.DoArmAction,
                                                execute_cb=self.execute,
                                                auto_start=False)
        rospy.wait_for_service('mavros/cmd/arming')
        self._state = rospy.wait_for_message('mavros/state',State)
        self.arm_srv = rospy.ServiceProxy('mavros/cmd/arming',CommandBool)
        self._as.start()
        rospy.loginfo('{0}: online'.format(self._action_name))

    def execute(self, goal):
        rospy.loginfo('{0}: received goal: {1}'.format(self._action_name,goal))
        state_sub = rospy.Subscriber('mavros/state',State,self.handle_state)
        r = rospy.Rate(1)
        res = CommandBoolResponse()
        tries = 0
        while not res.success:
            res = self.arm_srv(goal.arm_cmd)
#            self._result.succeeded = res.success
            tries += 1
            if tries >= 3:
                rospy.logwarn('{0}: failed to arm. Aborting'.format(self._action_name))
                self._as.set_aborted(self._result)
                return
            if res.success:
                timer = 5
                while (not self._state.armed and goal.arm_cmd) or (self._state.armed and not goal.arm_cmd):
                    rospy.loginfo('{0}: waiting for vehicle to respond'.format(self._action_name))
                    r.sleep()
                    if not timer:
                        rospy.logwarn('{0}: arm service successful, but vehicle did not respond'.format(self._action_name))
                        self._as.set_aborted(self._result)
                        return
                rospy.loginfo('{0}: {1} successful'.format(self._action_name,goal))
                self._result.final_state = self._state.armed
                self._as.set_succeeded(self._result)
            r.sleep()
        state_sub.unregister()
        rospy.loginfo('{0}: done executing goal'.format(self._action_name))

    def handle_state(self, data):
        self._state = data
        self._feedback.curr_status = self._state.armed
        if not self._as.is_active():
            return
        self._as.publish_feedback(self._feedback)

if __name__ == '__main__':
    rospy.init_node('arm_action_server')
    server = ArmServer('arm_action')
    rospy.spin()
