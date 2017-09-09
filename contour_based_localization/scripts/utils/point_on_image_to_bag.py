import rosbag
from geometry_msgs.msg import Pose2D

INPUT_BAG_FILE = r'/home/cear/data/bebop_top_view_1/canopies_600_messages.bag'
OUTPUT_BAG_FILE = r'/home/cear/data/bebop_top_view_1/pose.bag'
IMAGE_TOPIC = r'/bebop/image_raw'
POSE_TOPIC = r'/pose'

X = 508
Y = 100

if __name__ == '__main__':
    image_bag = rosbag.Bag(INPUT_BAG_FILE)
    pose_bag = rosbag.Bag(OUTPUT_BAG_FILE, 'w')
    i = 0
    for topic, msg, t in image_bag.read_messages(topics=IMAGE_TOPIC):
        pose_msg = Pose2D()
        pose_msg.x = int(X)
        pose_msg.y = int(Y)
        pose_bag.write(POSE_TOPIC, pose_msg, t)
        i += 1
    image_bag.close()
    pose_bag.close()