import cv2

IMAGE_PATH = r'/home/cear/data/panorama_extended_2/maps/map_canopies.pgm'

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print x,y

if __name__ == '__main__':
    image = cv2.imread(IMAGE_PATH)
    cv2.imshow('image', image)
    cv2.setMouseCallback('image', mouse_callback)

    while True:
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            print 'bye bye'
            break
