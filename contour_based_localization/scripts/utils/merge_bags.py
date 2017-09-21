import rosbag

IN_BAG_FILE1 = r'/home/cear/data/bebop_top_view_1/dynamic_pose_6_800_messages/pose.bag'
IN_BAG_FILE2 = r'/home/cear/data/bebop_top_view_1/canopies_800_messages.bag'
IN_BAG_FILE3 = r'/home/cear/data/bebop_top_view_1/trunks_800_messages.bag'
# IN_BAG_FILE1 = r'/home/cear/data/bebop_top_view_1/static_pose_1_100_messages/pose.bag'
# IN_BAG_FILE2 = r'/home/cear/data/bebop_top_view_1/trunks_100_images.bag'

OUT_BAG_FILE = r'/home/cear/data/bebop_top_view_1/dynamic_pose_6_800_messages/canopies_and_trunks_with_pose.bag'
# OUT_BAG_FILE = r'/home/cear/data/bebop_top_view_1/dynamic_pose_1_100_messages/trunks_with_pose.bag'

if __name__ == '__main__':
    in_bag1 = rosbag.Bag(IN_BAG_FILE1, 'r')
    in_bag2 = rosbag.Bag(IN_BAG_FILE2, 'r')
    in_bag3 = rosbag.Bag(IN_BAG_FILE3, 'r')
    merged_bag = rosbag.Bag(OUT_BAG_FILE, 'w')

    for (topic1, msg1, t1), (topic2, msg2, t2), (topic3, msg3, t3) in zip(in_bag1.read_messages(), in_bag2.read_messages(), in_bag3.read_messages()):
        merged_bag.write(topic1, msg1, t1)
        merged_bag.write(topic2, msg2, t1)
        merged_bag.write(topic3, msg3, t1)
    in_bag1.close()
    in_bag2.close()
    in_bag3.close()
    merged_bag.close()