#!/usr/bin/env python
from __future__ import print_function, division

import sys
import rospy
import cv2
import tf
from tf.transformations import euler_matrix as eul_mat
from tf.transformations import euler_from_quaternion as efq
import numpy as np
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image, CameraInfo, Imu
from cv_bridge import CvBridge, CvBridgeError

from math import sqrt
from collections import deque

class Image_Processor(object):

  def __init__(self):
    self.imu = self.ImuState()

    self.image_sub = rospy.Subscriber("/camera1/image_rect_color",Image,self.recv)
    self.imu_sub = rospy.Subscriber("/mavros/imu/data",Imu,self.update_orient)
    self.tag_track_sub = rospy.Subscriber("/detections/tracked",Bool,self.tag_cb)
    self.pose_sub = rospy.Subscriber('/mavros/local_position/pose',PoseStamped,self.pose_cb)

    self.track_pub = rospy.Publisher("/tracker/tracked",Bool,queue_size=20)
    self.heading_pub = rospy.Publisher("tracker/heading",TwistStamped,queue_size=10)
    self.state_pub = rospy.Publisher("/lezl/state",TwistStamped,queue_size=10)
    self.landing_pub = rospy.Publisher("/camera1/target_rect",Image,queue_size=20)
    self.camera_info_pub = rospy.Publisher("/landing_pad/camera_info",CameraInfo,queue_size=20)

    self.listener = tf.TransformListener()
    self.bridge = CvBridge()
    self.tracked = Bool()
    self.tag_tracked = Bool()
    self.tracked.data = False
    self.tag_tracked.data = False
    self.heading = TwistStamped()
    self.cam_info = rospy.wait_for_message('/camera1/camera_info',CameraInfo)
    self.K = np.matrix(self.cam_info.K).reshape((3,3))
    self.fy = self.K[1,1]

  def tag_cb(self,data):
      self.tag_tracked = data

  def recv(self,data):
    #Take image date from ROS, convert it, and display it in a cv2 frame
    im = self.bridge.imgmsg_to_cv2(data,"bgr8")
    mask,heading,ct,ci = self.detect_target(im)
    self.heading.header.stamp = rospy.Time.now()
    if heading is None:# or any([abs(x)>0.2 for x in [self.imu.dqx,self.imu.dqy,self.imu.dqz,self.imu.dqw]]):
        self.tracked.data = False
        cv2.imshow('from_pi',im)
        cv2.waitKey(3)
    else:
        im = cv2.bitwise_and(im,im,mask=mask)
        imline = cv2.line(im,(int(ci[0]),int(ci[1])),(int(ct[0]),int(ct[1])),(0,0,255))
        self.tracked.data = True
        vec_len = sqrt(heading[0]*heading[0] + heading[1]*heading[1])
        self.heading.twist.linear.x = heading[0]/vec_len
        self.heading.twist.linear.y = heading[1]/vec_len
#        self.heading.twist.linear.x = -self.dx
#        self.heading.twist.linear.y = -self.dy
        self.heading.twist.linear.z = self.dz
        cv2.imshow('from_pi',im)
        cv2.waitKey(3)
    self.track_pub.publish(self.tracked)
#    self.heading_pub.publish(self.heading)
#    self.camera_info_pub.publish(self.cam_info)

  def detect_target(self,data):
    hsv = cv2.cvtColor(data,cv2.COLOR_BGR2HSV)
    yel_lo = (20,100,100)
    yel_hi = (30,255,255)
    mask = cv2.inRange(hsv,yel_lo,yel_hi)
    _,cnts,heir = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
#    print(cnts)
    if not len(cnts):
        return data,None,None,None
    target = sorted(cnts,key=lambda c:cv2.contourArea(c))[-1]
    ellipse_rect = cv2.fitEllipse(target)
    if not abs(1 - (ellipse_rect[1][0] - ellipse_rect[1][1])) > 0.2:
        dist = self.dz - 0.1 #assume we're really close and we've lost the platform a little
    else:
        area = cv2.contourArea(target)
        diam = np.sqrt(4*area/np.pi)
        dist = 0.255*2*self.fy/diam
    self.dz = self.pose.position.z #cheat for now
#    self.dz = dist
#    print(self.dz,dist)
    if dist < 3:
        im = self.bridge.cv2_to_imgmsg(data, encoding="passthrough")
        self.landing_pub.publish(im)
    M = cv2.moments(target)
    if not M["m00"]:
        return mask,None, None,None
    Pi = np.matrix((M["m10"]/M["m00"],M["m01"]/M["m00"],1)).T
#    print(Pi.T)
    Pc = self.K.I*Pi
    Pc = Pc/Pc[2] * dist
#    print(Pc.T)
#    o = self.pose.orientation
#    q = [o.x,o.y,o.z,o.w]
#    r,p,y = efq(q)
#    R = np.matrix(eul_mat(y,p,r,axes='rzxy'))[:3,:3]
#    Pr = R*Pc
    self.dx = -Pc[0]
    self.dy = Pc[1]
    
    ct = ((M["m10"]/M["m00"]),(M["m01"]/M["m00"]))
    ci = ((data.shape[0]/2),(data.shape[1]/2))
    heading = (ct[0]-ci[0],ci[1]-ct[1])
##    self.dx = heading[0]*diam/(self.fy*2*0.255)
##    self.dy = heading[1]*diam/(self.fy*2*0.255)
#    self.dx = self.dz*np.tan(np.arctan(heading[0]/self.fy))
#    self.dy = self.dz*np.tan(np.arctan(heading[1]/self.fy))
    
    state = TwistStamped()
    state.header.stamp = rospy.Time.now()
    state.twist.linear.x = self.dx
    state.twist.linear.y = self.dy
    state.twist.linear.z = self.dz
    if not self.tag_tracked.data:
        self.state_pub.publish(state)
    else:
        pass
    return mask,heading,ct,ci

  def update_orient(self,imu_msg):
    self.imu(imu_msg.orientation)

  def pose_cb(self,msg):
    self.pose = msg.pose

  class ImuState(object):
    def __init__(self):
      self.id = 0
      self.ml = 200
      self.qx = deque(maxlen=self.ml)
      self.qy = deque(maxlen=self.ml)
      self.qz = deque(maxlen=self.ml)
      self.qw = deque(maxlen=self.ml)
      self.dqx = 0
      self.dqy = 0
      self.dqz = 0
      self.dqw = 0

    def __call__(self,quat):
      self.id += 1
      self.qx.append(quat.x)
      self.qy.append(quat.y)
      self.qz.append(quat.z)
      self.qw.append(quat.w)
      if self.id >= self.ml:
        self.dqx = abs(quat.x-sum(self.qx)/self.ml)
        self.dqy = abs(quat.y-sum(self.qy)/self.ml)
        self.dqz = abs(quat.z-sum(self.qz)/self.ml)
        self.dqw = abs(quat.w-sum(self.qw)/self.ml)




def main(args):
  rospy.init_node('image_converter', anonymous=True)
  #Start our image conversion object
  ic = Image_Processor()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
