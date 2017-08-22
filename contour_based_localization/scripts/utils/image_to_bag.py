import rosbag
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy.rostime as rostime
import time
import sys
import rospy
import numpy as np

# IMAGE_PATH = r'/home/cear/data/bebop_top_view_1/canopies.png'
# BAG_FILE = r'/home/cear/data/bebop_top_view_1/canopies_100_images.bag'
IMAGE_PATH = r'/home/cear/data/bebop_top_view_1/trunks.png'
BAG_FILE = r'/home/cear/data/bebop_top_view_1/trunks_100_images.bag'
NUM_OF_MESSAGES = 100
FPS = 30
FRAME_ID = '/bebop/camera_optical'
TOPIC = '/bebop/image_raw'

if __name__ == '__main__':
    rospy.init_node('image_to_topic')
    image = cv2.imread(IMAGE_PATH)
    image_bag = rosbag.Bag(BAG_FILE, 'w')
    bridge = CvBridge()
    for i in range(NUM_OF_MESSAGES):
        try:
            msg = bridge.cv2_to_imgmsg(image, encoding='bgr8')
        except CvBridgeError, e:
            print (e)
            sys.exit()
        msg.header.frame_id = FRAME_ID
        msg.height = np.size(image, 0)
        msg.width = np.size(image, 1)
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = np.size(image, 1) * 3
        t = rostime.Time.now()
        image_bag.write(TOPIC, msg, t)
        time.sleep(1.0/FPS)
    image_bag.close()