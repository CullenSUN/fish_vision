import cv2
import os
from opencv_image import *

class VideoProcessor:

    def __init__(self):
        self.previous_objects = None
        self.throttling_counter = 0

    def increase_counter(self):
        self.throttling_counter += 1
        if self.throttling_counter >= 5:
            self.throttling_counter = 0

    def callback_image(self, img):
        if self.throttling_counter % 5 != 0:
            self.increase_counter()
            return None

        if self.previous_objects is None:
            self.previous_objects = detect_objects(img)
            self.increase_counter()
            return None
        else:
            current_objects = detect_objects(img)
            print("current_objects", len(current_objects))
            obstacle_rects = detect_obstacles(self.previous_objects, current_objects)
            print("confirmed obstacles", len(obstacle_rects))
            self.previous_objects = current_objects
            self.increase_counter()
            return obstacle_rects

if __name__ == '__main__':
    cv2.namedWindow('Obstacles', cv2.WINDOW_NORMAL)

    script_path = os.path.dirname(os.path.realpath(__file__))
    img_path = os.path.join(script_path, 'videos/home_capture_1.h264')
    #img_path = os.path.join(script_path, 'videos/pool_capture_4.h264')

    processor = VideoProcessor()
    raw = cv2.VideoCapture(img_path)

    while(raw.isOpened()):
        # Capture frame-by-frame
        ret, img = raw.read()
        if not ret:
            break
        
        img = resize_image(img, 0.5)
        # print('resized dimensions: ', img.shape)

        detected_obstacles = processor.callback_image(img)
        for (rect, scale) in detected_obstacles or []: 
            x, y, w, h = rect
            red_color = min(255, 128*scale)
            cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, red_color), 2)
        cv2.imshow('Obstacles', img)

        # Press Q on keyboard to exit
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

    # When everything done, release the video capture object
    raw.release()
    
    # Closes all the frames
    cv2.destroyAllWindows()
