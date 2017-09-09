#!/usr/bin/env python

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Pose2D

class ContoursLaserScanGenerator(object):
    def __init__ (self):
        rospy.init_node('contours_laserscan_generator')

        self.laser_scale_factor = float(rospy.get_param('~laser_scale_factor'))
        self.image_fps = int(rospy.get_param('~image_fps'))
        self.downsample_rate = int(rospy.get_param('~downsample_rate', 2))
        self.laser_samples_number = int(rospy.get_param('~laser_samples_number', 400))
        self.laser_radius = float(rospy.get_param('~laser_radius_pixels', 150))
        self.laser_min_range = float(rospy.get_param('~laser_min_range_pixels', 20))
        self.laser_min_angle = float(rospy.get_param('~laser_min_angle_radians', -np.pi))
        self.laser_max_angle = float(rospy.get_param('~laser_max_angle_radians', np.pi))
        green_lower_hue_degrees = int(rospy.get_param('~green_lower_hue_degrees', 65))
        green_upper_hue_degrees = int(rospy.get_param('~green_upper_hue_degrees', 165))
        green_lower_saturation_percent = int(rospy.get_param('~green_lower_saturation_percent', 5))
        green_upper_saturation_percent = int(rospy.get_param('~green_upper_saturation_percent', 100))
        green_lower_value_percent = int(rospy.get_param('~green_lower_value_percent', 0))
        green_upper_value_percent = int(rospy.get_param('~green_upper_value_percent', 100))
        self.lower_color = np.array([green_lower_hue_degrees / 2, green_lower_saturation_percent * 255 / 100, green_lower_value_percent * 255 / 100])
        self.upper_color = np.array([green_upper_hue_degrees / 2, green_upper_saturation_percent * 255 / 100, green_upper_value_percent * 255 / 100])
        self.image_height = None
        self.image_width = None
        self.bridge = CvBridge()
        self.center_of_scan = None
        self.downsample_count = 0
        self.first_image_received = False

        rospy.Subscriber('/camera/image_raw', Image, self.image_raw_callback)
        rospy.Subscriber('/center_of_scan', Pose2D, self.center_of_scan_callback)
        self.laser_scan_pub = rospy.Publisher('contours_scan', LaserScan, queue_size=10)
        self.analyzed_image_pub = rospy.Publisher('analyzed_image', Image, queue_size=10)

    def image_raw_callback(self, msg):
        if not self.first_image_received:
            self.first_image_received = True
            self.image_height = msg.height
            self.image_width = msg.width
        self.downsample_count = (self.downsample_count + 1) % self.downsample_rate
        if self.downsample_count != 0:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError, e:
            print (e)
        self.generate_laser_scan(image)

    def center_of_scan_callback(self, msg):
        self.center_of_scan = msg
         
    def generate_laser_scan(self, image):
        center_of_scan = self.center_of_scan
        if center_of_scan is None:
            return

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        roi_height = self.laser_radius * 2 + 2
        roi_width = self.laser_radius * 2 + 2
        x_roi_range = np.clip([center_of_scan.x - roi_width / 2, center_of_scan.x + roi_width / 2], 0, self.image_width - 1)
        y_roi_range = np.clip([center_of_scan.y - roi_height / 2, center_of_scan.y + roi_height / 2], 0, self.image_height - 1)
        hsv = hsv[y_roi_range[0]:y_roi_range[1], x_roi_range[0]:x_roi_range[1]]
        mask = cv2.inRange(hsv, self.lower_color, self.upper_color)
        analyzed_image = image.copy()
        analyzed_image = analyzed_image[y_roi_range[0]:y_roi_range[1], x_roi_range[0]:x_roi_range[1]]
        contours,hierarchy = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(analyzed_image, contours, -1, (0, 255, 0), 3)

        laser_scan = LaserScan()
        laser_scan.header.stamp = rospy.rostime.Time.now()
        laser_scan.header.frame_id = 'contours_scan_link'
        laser_scan.angle_min = self.laser_min_angle
        laser_scan.angle_max = self.laser_max_angle
        laser_scan.angle_increment = (self.laser_max_angle - self.laser_min_angle) / self.laser_samples_number
        laser_scan.scan_time = self.image_fps / self.downsample_rate
        laser_scan.range_min = self.laser_min_range * self.laser_scale_factor
        laser_scan.range_max = self.laser_radius * self.laser_scale_factor

        center_of_scan_in_roi_x = int(center_of_scan.x - x_roi_range[0])
        center_of_scan_in_roi_y = int(center_of_scan.y - y_roi_range[0])
        scan_index = 0
        cv2.circle(analyzed_image,(center_of_scan_in_roi_x, center_of_scan_in_roi_y), radius=3, color=(255,0,255), thickness=2)
        laser_scan_ranges = np.full(self.laser_samples_number, np.nan)
        for theta in np.linspace(self.laser_min_angle, self.laser_max_angle, num=self.laser_samples_number):
            for r in np.linspace(self.laser_min_range, self.laser_radius, num=30):
                px =  int(np.round(center_of_scan_in_roi_x + r * np.cos(-theta)))
                py =  int(np.round(center_of_scan_in_roi_y + r * np.sin(-theta)))
                if px >= np.size(mask, 1) or px < 0 or py >= np.size(mask, 0) or py < 0:
                    break                    
                try:
                    if mask[py,px] == 255:
                        laser_scan_ranges[scan_index] = (np.sqrt((center_of_scan_in_roi_x-px)**2 + (center_of_scan_in_roi_y-py)**2)) * self.laser_scale_factor
                        cv2.circle(analyzed_image,(int(px),int(py)), radius=3, color=(255,255,0), thickness=2)
                        break
                except IndexError, e:
                    print (e)
            scan_index += 1
        laser_scan.ranges = laser_scan_ranges.tolist()
        self.laser_scan_pub.publish(laser_scan)
        ros_image_message = self.bridge.cv2_to_imgmsg(analyzed_image, encoding='bgr8')
        self.analyzed_image_pub.publish(ros_image_message)


if __name__ == '__main__':
    ContoursLaserScanGenerator()
    rospy.spin()