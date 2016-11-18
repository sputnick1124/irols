#!/usr/bin/env python
from __future__ import print_function, division

import sys
import rospy
import cv2
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from math import sqrt

class image_converter:

  def __init__(self):
    #Get our publisher ready
#    self.image_pub = rospy.Publisher("image_topic",Image,queue_size = 10)
    #Get CvBridge object ready. This will do the conversion cv2->ROS
    self.image_sub = rospy.Subscriber("/camera1/image_raw",Image,self.recv)
    self.track_pub = rospy.Publisher("/tracker/tracked",Bool,queue_size=20)
    self.heading_pub = rospy.Publisher("tracker/heading",TwistStamped,queue_size=10)
#    self.image_sub = rospy.Subscriber("/tracker/image",Image,self.recv)
    self.bridge = CvBridge()
    self.tracked = Bool()
    self.tracked.data = False
    self.heading = TwistStamped()
    #Start our videocapture device

  def send(self,data):
    #Take image date from cv2, convert it, and send it out to ROS
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(data, "rgb8"))
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
    M = cv2.moments(target)
    if not M["m00"]:
    	return mask,None, None,None
    ct = ((M["m10"]/M["m00"]),(M["m01"]/M["m00"]))
    ci = ((data.shape[0]/2),(data.shape[1]/2))
    heading = (ct[0]-ci[0],ci[1]-ct[1])
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
