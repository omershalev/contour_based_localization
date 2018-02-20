import rosbag
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy.rostime as rostime
import time
import sys
import rospy
import numpy as np

# IMAGE_PATH = r'/home/cear/data/panorama_extended_2/canopies_extended.jpg'
# BAG_FILE = r'/home/cear/data/panorama_extended_2/canopies_800_messages.bag'
IMAGE_PATH = r'/home/cear/data/panorama_extended_2/trunks.png'
BAG_FILE = r'/home/cear/data/panorama_extended_2/trunks_800_messages.bag'
NUM_OF_MESSAGES = 800
FPS = 30
FRAME_ID = '/bebop/camera_optical'
# TOPIC = '/bebop/image_raw'
TOPIC = '/estimated_trunks'

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