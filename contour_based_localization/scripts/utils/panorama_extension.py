import cv2
import numpy as np

INPUT_IMAGE = r'/home/cear/data/panorama_extended_16/canopies_photoshopped.jpg'
EXTENSION_RATIO = 1.0 / 6
OUTPUT_IMG = r'/home/cear/data/panorama_extended_16/canopies_photoshopped_extended.jpg'

if __name__ == '__main__':

    img = cv2.imread(INPUT_IMAGE)
    img_extended = np.tile(img, (3,3,1))
    x1 = int((1.0 / 3 - EXTENSION_RATIO / 3) * img_extended.shape[0])
    x2 = int((2.0 / 3 + EXTENSION_RATIO / 3) * img_extended.shape[0])
    y1 = int((1.0 / 3 - EXTENSION_RATIO / 3) * img_extended.shape[1])
    y2 = int((2.0 / 3 + EXTENSION_RATIO / 3) * img_extended.shape[1])
    img_final = img_extended[x1:x2,y1:y2]
    cv2.imwrite(OUTPUT_IMG, img_final)