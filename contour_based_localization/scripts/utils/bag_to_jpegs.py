import rosbag
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys

BAG_FILE = r'/home/cear/bags/trees_with_cam_calib_lavi_3.bag'
OUTPUT_FOLDER = r'/home/cear/bags/panorama_trees_with_cam_calib_lavi_3/'

if __name__ == '__main__':
    bag = rosbag.Bag(BAG_FILE, 'r')
    bridge = CvBridge()
    i = 0
    for topic, msg, t in bag.read_messages():
        try:
            image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError, e:
            print (e)
            sys.exit()
        cv2.imwrite(OUTPUT_FOLDER + 'img_' + str(i) + '.jpg', image)
        i += 1