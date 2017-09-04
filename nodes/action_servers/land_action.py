#!/usr/bin/env python
import rospy
import actionlib

import irols.msg

from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import SetMode

from geometry_msgs.msg import PoseStamped, Twist

class LandServer(object):
    _feedback = irols.msg.DoLandFeedback()
    _result = irols.msg.DoLandResult()
    
    def __init__(self,name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                irols.msg.DoLandAction,
                                                execute_cb=self.execute,
                                                auto_start=False)

#        rospy.wait_for_service('mavros/cmd/arming')
#        self.arm_srv = rospy.ServiceProxy('mavros/cmd/arming',CommandBool)
        self.arm_action = actionlib.SimpleActionClient('arm_action',
                                                       irols.msg.DoArmAction)
#        self.arm_action.feedback_cb = self.handle_arm_fb

        self.ext_state = rospy.wait_for_message('mavros/extended_state',
                                                 ExtendedState)
        self.ext_state_sub = rospy.Subscriber('mavros/extended_state',
                                              ExtendedState,
                                              self.handle_ext_state)
        self.ext_state_sub = rospy.Subscriber('mavros/state',
                                              State,
                                              self.handle_state)

        self.curr_pos = rospy.wait_for_message('mavros/local_position/pose',
                                               PoseStamped)
        self.pos_sub = rospy.Subscriber('mavros/local_position/pose',
                                        PoseStamped,
                                        self.handle_pose)

        self.vel_sp_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped',
                                          Twist,
                                          queue_size=1)
                                          
        self.set_mode_client = rospy.ServiceProxy('mavros/set_mode',
                                                  SetMode)

        self.land_twist = Twist()
        self.land_twist.linear.z = -2.5
                
        self.landed = False
        self.armed = True
        
        self._as.start()
        rospy.loginfo('{0}: online'.format(self._action_name))
        
    def execute(self,goal):
        rospy.loginfo('{0}: received land request'.format(self._action_name))
        self._feedback.height = self.curr_pos.pose.position.z
        self._as.publish_feedback(self._feedback)
        r = rospy.Rate(10)
        rospy.loginfo('{0}: landing'.format(self._action_name))
        while not self.landed:
            self.vel_sp_pub.publish(self.land_twist)
            r.sleep()
        rospy.loginfo('{0}: disarming'.format(self._action_name))
        self.arm_action.send_goal(irols.msg.DoArmGoal(arm_cmd=irols.msg.DoArmGoal.DISARM),
                                  done_cb=self.handle_arm_done)
        while self.armed:
            self.vel_sp_pub.publish(self.land_twist)
            r.sleep
#        wait_start = rospy.Time.now()
#        wait_duration = rospy.Duration(5)
#        while rospy.Time.now() - wait_start < wait_duration:
        while not self.mode == 'AUTO.LOITER':
            self.set_mode_client(custom_mode='AUTO.LOITER')
#            rospy.loginfo('{0}: {1} --> {2}'.format(self._action_name,self.mode,'AUTO.LOITER'))
            self.vel_sp_pub.publish(self.land_twist)
            r.sleep()
        self._result.success = True
        self._as.set_succeeded(self._result)
        rospy.loginfo('{0}: successfully landed and disarmed'.format(self._action_name))
    
    def handle_pose(self,data):
        self.curr_pos = data
    
    def handle_ext_state(self,data):
        self.landed = data.landed_state == ExtendedState.LANDED_STATE_ON_GROUND

    def handle_state(self,data):
        self.mode = data.mode

    def handle_arm_done(self,term_state,result):
        rospy.loginfo('{0}: listening to done result from arm_action'.format(self._action_name))
        rospy.loginfo('{0}: final state is {1}'.format(self._action_name,result.final_state))
        self.armed = result.final_state

if __name__ == '__main__':
    rospy.init_node('land_action_server')
    server = LandServer('land_action')
    rospy.spin()
