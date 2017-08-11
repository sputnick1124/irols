#!/usr/bin/env python
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from apriltags_ros.msg import AprilTagDetectionArray

class ApriltagEstimator(object):
    def __init__(self):
        self.tag_detection_sub = rospy.Subscriber(
                'tag_detections',
                AprilTagDetectionArray,
                self.handle_tags)
        self.odom_pub = rospy.Publisher(
                'apriltag_estimate',
                Odometry,
                queue_size=10)
        self.cov = [0.01, 0, 0, 0, 0, 0,
                    0, 0.01, 0, 0, 0, 0,
                    0, 0, 0.01, 0, 0, 0,
                    0, 0, 0, 0.1, 0, 0,
                    0, 0, 0, 0, 0.1, 0,
                    0, 0, 0, 0, 0, 0.1]

    def handle_tags(self,tag_array):
        if len(tag_array.detections) > 0:
            pose_w_cov = PoseWithCovariance(
                pose = tag_array.detections[0].pose.pose,
                covariance=self.cov)
            pose_w_cov.pose.position.x,pose_w_cov.pose.position.y = pose_w_cov.pose.position.y, pose_w_cov.pose.position.x
            odom = Odometry(pose=pose_w_cov)
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = 'pad'
            odom.child_frame_id = 'camera::camera_link'
            self.odom_pub.publish(odom)
        else:
            # no tag, so no estimate update
            pass

if __name__ == "__main__":
    rospy.init_node("apriltag_estimator")
    estimator = ApriltagEstimator()
    rospy.spin()
