#!/usr/bin/env python
import rospy
import cv2
import numpy as np

from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import Image, CameraInfo

from dynamic_reconfigure.server import Server
from irols.cfg import moveCamConfig
from cv_bridge import CvBridge

from tf.transformations import euler_matrix as eul_mat
from tf.transformations import quaternion_from_euler as qfe
from tf.transformations import euler_from_quaternion as efq
from numpy.linalg import inv
from math import sqrt

class CameraMover(object):
    def __init__(self):
        self.model_pub = rospy.Publisher('/gazebo/set_model_state',
                                         ModelState,
                                         queue_size=1)
        self.srv = Server(moveCamConfig,self.set_model_state)

    def set_model_state(self,config,level):
        model_state = ModelState(model_name='lezl_cam')
        x = config['x']
        y = config['y']
        z = config['z']
        phi = config['phi']
        theta = config['theta']
        psi = -config['psi']
        model_state.pose.position.x = x
        model_state.pose.position.y = y
        model_state.pose.position.z = z
#        R = np.matrix(eul_mat(0,1.57,0,'rzxy'))
        q = qfe(phi,theta,psi,'sxyz')
        model_state.pose.orientation.x = q[0]
        model_state.pose.orientation.y = q[1]
        model_state.pose.orientation.z = q[2]
        model_state.pose.orientation.w = q[3]
        self.model_pub.publish(model_state)
        self.pose = model_state.pose
        return config

class Homographer(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.cam_info = rospy.wait_for_message('/camera1/camera_info',CameraInfo)
        self.K = np.matrix(self.cam_info.K).reshape((3,3))
        self.fy = self.K[1,1]
        self.image_sub = rospy.Subscriber("/camera1/image_rect_color",Image,self.recv)
        self.cam_mover = CameraMover()

    def recv(self,data):
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

    def detect_target(self,data):
        hsv = cv2.cvtColor(data,cv2.COLOR_BGR2HSV)
        yel_lo = (20,100,100)
        yel_hi = (30,255,255)
        mask = cv2.inRange(hsv,yel_lo,yel_hi)
        _,cnts,heir = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        #print(cnts)
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
        self.dz = self.cam_mover.pose.position.z #cheat for now
        #self.dz = dist
        #print(self.dz,diam)
        M = cv2.moments(target)
        if not M["m00"]:
            return mask,None, None,None
        Pi = np.matrix((M["m10"]/M["m00"],M["m01"]/M["m00"],1)).T # point in image coords
        Pc = self.K.I*Pi # point in world units (still on sensor)
        Pc = Pc/Pc[2] * dist # point projected onto ground plane
        Pc[1] *= -1 # account for image coordinates astarting in top left
        o = self.cam_mover.pose.orientation
        q = [o.x,o.y,o.z,o.w]
        r,p,y = efq(q,'sxyz')
#        p -= 1.57
        #R = np.matrix(eul_mat(y,p,-r,axes='rzxy'))[:3,:3]
        Rcam_body = np.matrix(eul_mat(0,0,-1.57))[:3,:3]
        Rbody_inert = np.matrix(eul_mat(r,p,-y))[:3,:3]
#        L = np.diag([1,1,1])
#        Pr = R.T*L*Pc
        Pr = Rbody_inert*Rcam_body.T*Pc
        self.dx = Pr[0]
        self.dy = -Pr[1]
        print('dx',float(Pr[0]),' dy',float(Pr[1]),'rpy',r,p,-y)
        ct = ((M["m10"]/M["m00"]),(M["m01"]/M["m00"]))
        ci = ((data.shape[0]/2),(data.shape[1]/2))
        heading = (ct[0]-ci[0],ci[1]-ct[1])
    ##    self.dx = heading[0]*diam/(self.fy*2*0.255)
    ##    self.dy = heading[1]*diam/(self.fy*2*0.255)
    #    self.dx = self.dz*np.tan(np.arctan(heading[0]/self.fy))
    #    self.dy = self.dz*np.tan(np.arctan(heading[1]/self.fy))
        return mask,heading,ct,ci

if __name__ == "__main__":
    rospy.init_node('homography')
    homographer = Homographer()
    rospy.spin()
