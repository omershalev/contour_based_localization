#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped

FRAME_HEIGHT = 480

class InitialposePub(object):
    def __init__(self):
        rospy.init_node('initialpose_publisher')
        rospy.Subscriber('/com', Pose2D, self.com_callback)
        self.first_message_received = False

    def com_callback(self, msg):
        if not self.first_message_received:
            self.first_message_received = True
            pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.pose.position.x = msg.x * 0.012562814
            pose_msg.pose.pose.position.y = (FRAME_HEIGHT - msg.y) * 0.012562814
            pose_msg.pose.pose.orientation.w = 1
            pose_msg.pose.covariance = [0.0000001, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0000001, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0000000001]
            rospy.sleep(0.5)
            pose_pub.publish(pose_msg)

if __name__ == '__main__':
    InitialposePub()
    rospy.spin()