import rosbag

# IN_BAG_FILE1 = r'/home/cear/data/bebop_top_view_1/static_pose_1_100_messages/pose.bag'
# IN_BAG_FILE2 = r'/home/cear/data/bebop_top_view_1/canopies_100_messages.bag'
IN_BAG_FILE1 = r'/home/cear/data/bebop_top_view_1/static_pose_1_100_messages/pose.bag'
IN_BAG_FILE2 = r'/home/cear/data/bebop_top_view_1/trunks_100_images.bag'

OUT_BAG_FILE = r'/home/cear/data/bebop_top_view_1/static_pose_1_100_messages/trunks_with_pose.bag'

if __name__ == '__main__':
    in_bag1 = rosbag.Bag(IN_BAG_FILE1, 'r')
    in_bag2 = rosbag.Bag(IN_BAG_FILE2, 'r')
    merged_bag = rosbag.Bag(OUT_BAG_FILE, 'w')

    for topic, msg, t in in_bag1.read_messages():
        merged_bag.write(topic, msg, t)
    for topic, msg, t in in_bag2.read_messages():
        merged_bag.write(topic, msg, t)

    in_bag1.close()
    in_bag2.close()
    merged_bag.close()