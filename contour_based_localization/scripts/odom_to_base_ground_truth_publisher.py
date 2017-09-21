#!/usr/bin/env python

import rospy
import tf.transformations
from geometry_msgs.msg import Pose2D

class TfPub(object):
    def __init__(self):
        rospy.init_node('odom_to_base_tf_publisher')
        self.resolution = float(rospy.get_param('~resolution'))
        self.base_frame_id = rospy.get_param('~base_frame_id')
        self.odom_frame_id = rospy.get_param('~odom_frame_id')
        rospy.Subscriber('/pose', Pose2D, self.pose_callback)
        self.first_message_received = False
        self.x_init = None
        self.y_init = None

    def pose_callback(self, msg):
        if not self.first_message_received:
            self.x_init = msg.x
            self.y_init = msg.y
            self.first_message_received = True
        br = tf.TransformBroadcaster()
        x = (msg.x - self.x_init) * self.resolution
        y = (-(msg.y - self.y_init)) * self.resolution
        br.sendTransform((x, y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                         rospy.Time.now(),
                         self.base_frame_id,
                         self.odom_frame_id)

if __name__ == '__main__':
    TfPub()
    rospy.spin()