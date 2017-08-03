#!/usr/bin/env python
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image, CameraInfo, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance

from cv_bridge import CvBridge

from tf.transformations import euler_matrix as eul_mat
from tf.transformations import euler_from_quaternion as efq

class VisEstimator(object):
    def __init__(self):
        rospy.loginfo("Waiting for camera publishers")
        self.cam_info = rospy.wait_for_message('camera1/camera_info',CameraInfo)
        rospy.loginfo("Receiving camera image")
        
        rospy.loginfo("Waiting for imu data")
        rospy.wait_for_message('mavros/imu/data',Imu)
        rospy.loginfo("Receiving imu data")
        
        # Subscribers
        self.image_sub = rospy.Subscriber(
            'camera1/image_rect_color',
            Image,
            self.handle_image)
        self.imu_sub = rospy.Subscriber(
            'mavros/imu/data',
            Imu,
            self.handle_imu)
        
        # Publisher
        self.odom_pub = rospy.Publisher(
            'vision_estimate',
            Odometry,
            queue_size=10)
        self.cov = [0.25, 0, 0, 0, 0, 0,
                    0, 0.25, 0, 0, 0, 0,
                    0, 0, 0.5, 0, 0, 0,
                    0, 0, 0, 5, 0, 0,
                    0, 0, 0, 0, 5, 0,
                    0, 0, 0, 0, 0, 5]
                
        # Set up camera stuff
        self.bridge = CvBridge()
        self.K = np.matrix(self.cam_info.K).reshape((3,3))
        self.fy = self.K[1,1]

    def handle_image(self,data):
        #Take image date from ROS, convert it, and display it in a cv2 frame
        im = self.bridge.imgmsg_to_cv2(data,"bgr8")
        mask,heading,ct,ci = self.detect_target(im)
        if heading is None:# or any([abs(x)>0.2 for x in [self.imu.dqx,self.imu.dqy,self.imu.dqz,self.imu.dqw]]):
            cv2.imshow('from_cam',im)
            cv2.waitKey(3)
        else:
            im = cv2.bitwise_and(im,im,mask=mask)
            cv2.line(im,(int(ci[0]),int(ci[1])),(int(ct[0]),int(ct[1])),(0,0,255))
            cv2.imshow('from_cam',im)
            cv2.waitKey(3)

    def handle_imu(self,data):
        self.orientation = data.orientation

    def detect_target(self,data):
        # find yellow blob
        hsv = cv2.cvtColor(data,cv2.COLOR_BGR2HSV)
        yel_lo = (20,100,100)
        yel_hi = (30,255,255)
        mask = cv2.inRange(hsv,yel_lo,yel_hi)
        
        _,cnts,heir = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        if not len(cnts): # no yellow(target) in image
            return data,None,None,None
        target = sorted(cnts,key=lambda c:cv2.contourArea(c))[-1]
        leftmost = tuple(target[target[:,:,0].argmin()][0])
        rightmost = tuple(target[target[:,:,0].argmax()][0])
        topmost = tuple(target[target[:,:,1].argmin()][0])
        bottommost = tuple(target[target[:,:,1].argmax()][0])
        if (leftmost == 0 or
            topmost == 0 or 
            rightmost == self.cam_info.width or
            bottommost == self.cam_info.height): #target is at least partially occluded by edge of frame
            return data,None,None,None
        try:
            ellipse_rect = cv2.fitEllipse(target)
        except: #FIXME: need to capture only OpenCV Error
        #error: OpenCV Error: Incorrect size of input array (There should be at least 5 points to fit the ellipse) in fitEllipse
            return data,None,None,None
        
        #TODO: handle case where we are too close to see the platform and also when we just can't see a platform at all
        if not abs(1 - (ellipse_rect[1][0] - ellipse_rect[1][1])) > 0.2:
            dist = self.dz - 0.1 #assume we're really close and we've lost the platform a little
        else:
            area = cv2.contourArea(target)
            diam = np.sqrt(4*area/np.pi)
            dist = 0.255*2*self.fy/diam # actual radius of landing pad = 255mm
        dz = dist # not a very good guess, especially if we're tilted
        
        M = cv2.moments(target)
        if not M["m00"]:
            return mask,None, None,None
        Pi = np.matrix((M["m10"]/M["m00"],M["m01"]/M["m00"],1)).T # point in image coords
        Pc = self.K.I*Pi # point in world units (still on sensor)
        Pc = Pc/Pc[2] * dist # point projected onto ground plane
        Pc[1] *= -1 # account for image coordinates astarting in top left
        o = self.orientation
        q = [o.x,o.y,o.z,o.w]
        r,p,y = efq(q,'sxyz')
        Rcam_body = np.matrix(eul_mat(3.14,0,0))[:3,:3]
        Rbody_inert = np.matrix(eul_mat(r,p,-y))[:3,:3]
        Pr = Rbody_inert*Rcam_body.T*Pc
        dx = Pr[0]
        dy = -Pr[1]
        #print('dx',float(Pr[0]),' dy',float(Pr[1]),'rpy',r,p,-y)
        
        # build message for estimator
        pose = Pose()
        pose.position.x = dx
        pose.position.y = dy
        pose.position.z = dz
        pose.orientation = o
        pose_w_cov = PoseWithCovariance(
            pose=pose,
            covariance=self.cov)
        odom = Odometry(pose=pose_w_cov)
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'pad'
        odom.child_frame_id = 'camera::camera_link'
        self.odom_pub.publish(odom)
        
        ci = ((data.shape[0]/2),(data.shape[1]/2))
        heading = (Pi[0]-ci[0],ci[1]-Pi[1])
        return mask,heading,tuple(Pi[:2]),ci

if __name__ == "__main__":
    rospy.init_node('vision_estimator')
    estimator = VisEstimator()
    rospy.spin()
