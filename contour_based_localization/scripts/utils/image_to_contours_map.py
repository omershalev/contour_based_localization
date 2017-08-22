import cv2
import numpy as np
import yaml

COLOR_LOWER_HUE_DEGREES = 65
COLOR_LOWER_SAT_PERCENT = 5
COLOR_LOWER_VAL_PERCENT = 0
COLOR_UPPER_HUE_DEGREES = 165
COLOR_UPPER_SAT_PERCENT = 100
COLOR_UPPER_VAL_PERCENT = 100

SCALE_FACTOR = 0.0125

IMAGE_PATH = r'/home/cear/data/bebop_top_view_1/snapshot.png'
# IMAGE_PATH = r'/home/cear/data/bebop_top_view_1/trunks.png'
PGM_PATH = r'/home/cear/contour_localization_ws/src/contour_based_localization/contour_based_localization/maps/map.pgm'
YAML_PATH = r'/home/cear/contour_localization_ws/src/contour_based_localization/contour_based_localization/maps/map.yaml'


if __name__ == '__main__':

    image = cv2.imread(IMAGE_PATH)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_color = np.array([COLOR_LOWER_HUE_DEGREES/2,COLOR_LOWER_SAT_PERCENT*255/100,COLOR_LOWER_VAL_PERCENT*255/100])
    upper_color = np.array([COLOR_UPPER_HUE_DEGREES/2,COLOR_UPPER_SAT_PERCENT*255/100,COLOR_UPPER_VAL_PERCENT*255/100])
    mask = cv2.inRange(hsv, lower_color, upper_color)

    contours_image = np.full((np.size(image, 0), np.size(image, 1)), 0)
    contours, hierarchy = cv2.findContours(mask.copy(), 1, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(contours_image, contours, -1, 128, -1)
    cv2.drawContours(contours_image, contours, -1, 255, 3)
    cv2.imwrite(PGM_PATH, contours_image)

    yaml_content = {'image' : PGM_PATH.split('/')[-1],
                    'resolution' : SCALE_FACTOR,
                    'origin' : [0.0, 0.0, 0.0],
                    'negate' : 1,
                    'occupied_thresh' : 0.9,
                    'free_thresh' : 0.1}
    with open(YAML_PATH, 'w') as yaml_file:
        yaml.dump(yaml_content, yaml_file)