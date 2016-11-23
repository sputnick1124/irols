#!/usr/bin/env python
from __future__ import print_function, division

import sys
import rospy
import cv2
import tf
import numpy as np
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

from math import sqrt

class image_converter:

  def __init__(self):
    #Get our publisher ready
#    self.image_pub = rospy.Publisher("image_topic",Image,queue_size = 10)
    #Get CvBridge object ready. This will do the conversion cv2->ROS
    self.image_sub = rospy.Subscriber("/camera1/image_rect",Image,self.recv)
#    self.landing_sub = rospy.Subscriber("/landing_pad/image_rect",Image,self.find_tag)
    self.camera_info_sub = rospy.Subscriber("/camera1/camera_info",CameraInfo,self.get_cam_info)
    self.track_pub = rospy.Publisher("/tracker/tracked",Bool,queue_size=20)
    self.heading_pub = rospy.Publisher("tracker/heading",TwistStamped,queue_size=10)
    self.landing_pub = rospy.Publisher("/landing_pad/image_rect",Image,queue_size=20)
    self.listener = tf.TransformListener()
#    self.image_sub = rospy.Subscriber("/tracker/image",Image,self.recv)
    self.bridge = CvBridge()
    self.tracked = Bool()
    self.tracked.data = False
    self.heading = TwistStamped()
#    self.cam_info = CameraInfo()
    #Start our videocapture device

  def get_cam_info(self,data):
    #Take image date from cv2, convert it, and send it out to ROS
#    self.cam_info = data
    width = data.width
    self.fy = data.K[0] # pixel scaled focal length
    fov = 0.84479790990252 #field of view
    f =  width/(2*np.tan(fov/2))# focal length of camera
    self.im_dist = f*f/self.fy #distance to object = perceived width * im_dist
    self.camera_info_sub.unregister()

  def send(self,data):
    #Take image date from cv2, convert it, and send it out to ROS
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(data, "rgb8"))
    except CvBridgeError as e:
      print(e)

  def find_tag(self,data):
    #Take image date from cv2, convert it, and send it out to ROS
    try:
      self.landing_pub.publish(self.bridge.cv2_to_imgmsg(data, "rgb8"))
    except CvBridgeError as e:
      print(e)

  def recv(self,data):
    #Take image date from ROS, convert it, and display it in a cv2 frame
    im = self.bridge.imgmsg_to_cv2(data,"bgr8")
    mask,heading,ct,ci = self.detect_target(im)
    self.heading.header.stamp = rospy.Time.now()
    if heading is None:
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
	    cv2.imshow('from_pi',im)
	    cv2.waitKey(3)
    self.track_pub.publish(self.tracked)
    self.heading_pub.publish(self.heading)

  def detect_target(self,data):
    hsv = cv2.cvtColor(data,cv2.COLOR_BGR2HSV)
    yel_lo = (20,100,100)
    yel_hi = (30,255,255)
    mask = cv2.inRange(hsv,yel_lo,yel_hi)
    cnts = cv2.findContours(mask,1,2)[0]
    if not len(cnts):
    	return data,None,None,None
    target = sorted(cnts,key=lambda c:cv2.contourArea(c))[-1]
    area = cv2.contourArea(target)
    diam = np.sqrt(4*area/np.pi)
    dist = 0.255*2*self.fy/diam
    self.dz = dist
    print(self.dz)
    if dist < 2:
    	im = self.bridge.cv2_to_imgmsg(data, encoding="passthrough")
    	self.landing_pub.publish(im)
    M = cv2.moments(target)
    if not M["m00"]:
    	return mask,None, None,None
    ct = ((M["m10"]/M["m00"]),(M["m01"]/M["m00"]))
    ci = ((data.shape[0]/2),(data.shape[1]/2))
    heading = (ct[0]-ci[0],ci[1]-ct[1])
    self.dx = heading[0]*diam/(2*0.255)
    self.dy = heading[1]*diam/(2*0.255)
    return mask,heading,ct,ci

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  #Start our image conversion object
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
