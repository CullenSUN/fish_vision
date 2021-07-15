import cv2
import os
from opencv_image import *

if __name__ == '__main__':
    cv2.namedWindow('Contours', cv2.WINDOW_NORMAL)

    script_path = os.path.dirname(os.path.realpath(__file__))
    #img_path = os.path.join(script_path, 'videos/pool_test_1.MOV')
    img_path = os.path.join(script_path, 'videos/pi_capture_2.h264')

    raw = cv2.VideoCapture(img_path)

    # Read until video is completed
    while(raw.isOpened()):
        
        # Capture frame-by-frame
        ret, img = raw.read()
        if not ret:
            break

        print('Original Dimensions : ', img.shape)
        img = resize_image(img, 0.5)
        print('Resized Dimensions : ', img.shape)

        #rotated = cv2.rotate(resized, cv2.ROTATE_90_COUNTERCLOCKWISE)

        process_canny(img)

        # Press Q on keyboard to  exit
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    
    
    # When everything done, release 
    # the video capture object
    raw.release()
    
    # Closes all the frames
    cv2.destroyAllWindows()
