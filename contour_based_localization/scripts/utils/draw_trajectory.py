import cv2
import rosbag
import os

IMAGE_PATH = r'/home/cear/data/panorama_extended_16/canopies_extended.jpg'
IMAGE_PATH = r'/home/cear/data/panorama_extended_16/canopies_photoshopped_extended.jpg'
# IMAGE_PATH = r'/home/cear/data/panorama_extended_16/maps/map_canopies.pgm'

POSE_BAG_PATH_1 = r'/home/cear/data/panorama_extended_16/fork/pose.bag'
POSE_BAG_PATH_2 = r'/home/cear/data/panorama_extended_16/snake/pose.bag'
POSE_BAG_PATH_3 = r'/home/cear/data/panorama_extended_16/random_walk/pose.bag'

OUTPUT_DIR = r'/home/cear/Pictures'
OUTPUT_FILE = r'on_panorama'

DOWNSAMPLE_RATE = 10

THREE_TOGETHER = True

if __name__ == '__main__':
    img = cv2.imread(IMAGE_PATH)
    new_img = img.copy()
    bag_idx = 0
    bag_idx_to_color = {0 : (255,70,0), 1 : (0,255,255), 2: (0,255,0)} # blue, yellow, green
    for bag_path in [POSE_BAG_PATH_1, POSE_BAG_PATH_2, POSE_BAG_PATH_3]:
        if not THREE_TOGETHER:
            new_img = img.copy()
        pose_bag = rosbag.Bag(bag_path)
        idx = 0
        points = []
        for _, msg, _ in pose_bag.read_messages(topics='/pose'):
            if idx == 0:
                points.append((int(msg.x), int(msg.y)))
            idx = (idx + 1) % DOWNSAMPLE_RATE
        for i in range(len(points)-1):
            cv2.line(new_img, points[i], points[i+1], bag_idx_to_color[bag_idx],2)
        bag_idx += 1
        if not THREE_TOGETHER:
            cv2.imshow('new image', new_img)
            cv2.waitKey()
            cv2.imwrite(os.path.join(OUTPUT_DIR, OUTPUT_FILE + str(bag_idx) + '.jpg'), new_img)
    if THREE_TOGETHER:
        cv2.imshow('new image', new_img)
        cv2.waitKey()
        cv2.imwrite(os.path.join(OUTPUT_DIR, OUTPUT_FILE + '.jpg'), new_img)