#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  def __init__(self):
    #Get our publisher ready
#    self.image_pub = rospy.Publisher("image_topic",Image,queue_size = 10)
    #Get CvBridge object ready. This will do the conversion cv2->ROS
    self.image_sub = rospy.Subscriber("/camera1/image_raw",Image,self.recv)
#    self.image_sub = rospy.Subscriber("/tracker/image",Image,self.recv)
    self.bridge = CvBridge()
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
    cv2.imshow('from_pi',im)
    cv2.waitKey(3)


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
