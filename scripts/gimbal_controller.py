#!/usr/bin/env python
import rospy

from mavros_msgs.srv import CommandLong

DO_MOUNT_CTL = 205
NULL = 0
MNT_MODE = 2 #set PRY commands instead of Lat/Lon

class GimbalController(object):
    def __init__(self):
        self.cmd_srv = rospy.ServiceProxy('mavros/cmd/command',CommandLong)
    
    def __call__(self,r=0,p=-90,y=0):
        self.cmd_srv(
            broadcast=False,
            command=DO_MOUNT_CTL,
            param1=p,
            param2=r,
            param3=y,
            param4=NULL,
            param5=NULL,
            param6=NULL,
            param7=MNT_MODE)