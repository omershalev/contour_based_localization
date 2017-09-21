import cv2
import rosbag
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose2D

INPUT_BAG_FILE = r'/home/cear/data/bebop_top_view_1/canopies_300_messages.bag'
OUTPUT_BAG_FILE = r'/home/cear/data/bebop_top_view_1/pose.bag'
IMAGE_TOPIC = r'/bebop/image_raw'
POSE_TOPIC = r'/pose'

class ManualTaggingPose(object):
    def __init__(self):
        self.image_bag = rosbag.Bag(INPUT_BAG_FILE)
        self.pose_bag = rosbag.Bag(OUTPUT_BAG_FILE, 'w')
        self.bridge = CvBridge()

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.p = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self.pose2d_msg = Pose2D()
            self.pose2d_msg.x = self.p[0]
            self.pose2d_msg.y = self.p[1]
            cv2.circle(self.image, self.p, 3, (0, 255, 0), 2)

    def play_bag(self):
        frame_number = 0
        for topic, msg, t in self.image_bag.read_messages(topics=IMAGE_TOPIC):
            print 'frame_number = ' + str(frame_number)
            frame_number += 1
            try:
                self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except CvBridgeError, e:
                print(e)
            clone = self.image.copy()

            cv2.namedWindow('image')
            cv2.setMouseCallback('image', self.mouse_callback)

            while True:
                cv2.imshow('image', self.image)
                key = cv2.waitKey(1) & 0xFF

                # if the 'r' key is pressed, reset the displayed image
                if key == ord('r'):
                    self.image = clone.copy()

                # if the 'c' key is pressed, confirm the selection
                elif key == ord('c'):
                    self.pose_bag.write(POSE_TOPIC, self.pose2d_msg, t)
                    cv2.destroyAllWindows()
                    break

                # if the 'n' key is pressed, no selection is done for this frame
                elif key == ord('n'):
                    cv2.destroyAllWindows()
                    break
        self.pose_bag.close()


if __name__ == '__main__':
    mt = ManualTaggingPose()
    mt.play_bag()