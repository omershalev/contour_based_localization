#!/usr/bin/env python

import rospy
import tf.transformations
from geometry_msgs.msg import Pose2D
import numpy as np

class TfPub(object):
    def __init__(self):
        rospy.init_node('odom_to_base_tf_publisher')
        self.resolution = float(rospy.get_param('~resolution'))
        self.base_frame_id = rospy.get_param('~base_frame_id')
        self.odom_frame_id = rospy.get_param('~odom_frame_id')
        self.duplicate_transform = rospy.get_param('~duplicate_transform', 'false')
        if self.duplicate_transform:
            self.duplicate_source = rospy.get_param('~duplicate_source')
            self.duplicate_target = rospy.get_param('~duplicate_target')
            rate = rospy.Rate(8)
            listener = tf.TransformListener()
            br = tf.TransformBroadcaster()
            while not rospy.is_shutdown():
                try:
                    (trans, rot) = listener.lookupTransform(self.duplicate_source, self.duplicate_target, rospy.Time(0))
                    br.sendTransform(trans, rot, rospy.Time.now(), self.base_frame_id, self.odom_frame_id)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
                rate.sleep()
        else:
            self.add_noise = rospy.get_param('~add_noise', 'false')
            rospy.Subscriber('/pose', Pose2D, self.pose_callback)
            self.first_message_received = False
            self.x_init = None
            self.y_init = None
            self.prev_x = None
            self.prev_y = None


    def pose_callback(self, msg):
        if not self.first_message_received:
            self.x_init = msg.x
            self.y_init = msg.y
            self.prev_x = msg.x
            self.prev_y = msg.y
            self.prev_broadcast_x = 0
            self.prev_broadcast_y = 0
            self.first_message_received = True
            return
        br = tf.TransformBroadcaster()
        x = (msg.x - self.x_init) * self.resolution
        y = (-(msg.y - self.y_init)) * self.resolution
        delta_x = x - self.prev_x * self.resolution
        delta_y = y - self.prev_y * self.resolution
        if self.add_noise:
            new_broadcast_x = self.prev_broadcast_x + delta_x + np.random.normal(0,0.3)
            new_broadcast_y = self.prev_broadcast_y + delta_y + np.random.normal(0,0.3)
        else:
            new_broadcast_x = self.prev_broadcast_x + delta_x
            new_broadcast_y = self.prev_broadcast_y + delta_y
        br.sendTransform((new_broadcast_x, new_broadcast_y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                         rospy.Time.now(),
                         self.base_frame_id,
                         self.odom_frame_id)

if __name__ == '__main__':
    TfPub()
    rospy.spin()