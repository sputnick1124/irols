#!/usr/bin/env python
import rospy
import actionlib

import irols.msg

from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolResponse


class DoArmServer(object):
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
#        self.state_sub = rospy.Subscriber('mavros/state',State,self.handle_state)
        self._as.start()

    def execute(self, goal):
        state_sub = rospy.Subscriber('mavros/state',State,self.handle_state)
        r = rospy.Rate(1)
        res = CommandBoolResponse()
        tries = 0
        while not res.success:
            res = self.arm_srv(goal.arm_cmd)
            self._result.succeeded = res.success
            tries += 1
            if tries >= 3:
                self._as.set_aborted(self._result)
            if res.success:
                self._as.set_succeeded(self._result)
            r.sleep()
        state_sub.unregister()

    def handle_state(self, data):
        self._state = data
        self._feedback.curr_status = self._state.armed
        if not self._as.is_active():
            return
        self._as.publish_feedback(self._feedback)

if __name__ == '__main__':
    rospy.init_node('arm_action_server')
    server = DoArmServer('arm_action')
    rospy.spin()