import cv2
import numpy as np
import random

SCALE_FACTOR = 0.0125

### bebop image ###
# WIDTH = 856
# HEIGHT = 480
# TRUNK_RADIUS = 7

### panorama 1 ###
# WIDTH = 848
# HEIGHT = 796
# TRUNK_RADIUS = 10

### panorama extended 2 ###
WIDTH = 1131
HEIGHT = 1061
TRUNK_RADIUS = 10


BACKGROUND_COLOR = (9, 59, 125)
TRUNK_COLOR = (0,147,54)

TRUNKS_LIST = r'/home/cear/data/panorama_extended_2/trunk_locations'
OUT_IMAGE_PATH = r'/home/cear/data/panorama_extended_2/trunks.png'

RANDOM = True

if __name__ == '__main__':

    f = open(TRUNKS_LIST)
    trunks_image = np.full((HEIGHT, WIDTH, 3), 0)
    trunks_image[:] = BACKGROUND_COLOR

    for line in f.read().splitlines():
        x, y = line.split(' ')
        if RANDOM:
            r = TRUNK_RADIUS + random.uniform(0, 3)
        else:
            r = TRUNK_RADIUS
        cv2.circle(trunks_image, (int(x),int(y)), radius=int(r), color=TRUNK_COLOR, thickness=-1)
        cv2.circle(trunks_image, (int(x),int(y)), radius=int(r), color=TRUNK_COLOR, thickness=3)

    cv2.imwrite(OUT_IMAGE_PATH, trunks_image)

    f.close()