#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from sensor_msgs.msg import Image

class InitialposePub(object):
    def __init__(self):
        rospy.init_node('initialpose_publisher')
        rospy.Subscriber('/pose', Pose2D, self.pose_callback)
        rospy.Subscriber('/camera/image_raw', Image, self.image_raw_callback)
        self.first_message_received = False
        self.resolution = float(rospy.get_param('~resolution'))
        self.variance_x = float(rospy.get_param('~laser_samples_number', 1e-7))
        self.variance_y = float(rospy.get_param('~laser_samples_number', 1e-7))
        self.variance_theta = float(rospy.get_param('~laser_samples_number', 1e-10))
        self.image_height = None

    def image_raw_callback(self, msg):
        if self.image_height is None:
            self.image_height = msg.height

    def pose_callback(self, msg):
        if self.first_message_received == False and self.image_height is not None:
            self.first_message_received = True
            pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.pose.position.x = msg.x * self.resolution
            pose_msg.pose.pose.position.y = (self.image_height - msg.y) * self.resolution
            pose_msg.pose.pose.orientation.w = 1
            pose_msg.pose.covariance = [self.variance_x, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, self.variance_y, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, self.variance_theta]
            rospy.sleep(0.5)
            pose_pub.publish(pose_msg)

if __name__ == '__main__':
    InitialposePub()
    rospy.spin()