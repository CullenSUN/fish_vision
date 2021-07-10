import cv2
import os
from opencv_image import *

if __name__ == '__main__':
    cv2.namedWindow('Contours', cv2.WINDOW_NORMAL)

    script_path = os.path.dirname(os.path.realpath(__file__))
    img_path = os.path.join(script_path, 'videos/turtle.mov')
    raw = cv2.VideoCapture(img_path)

    # Read until video is completed
    while(raw.isOpened()):
        
        # Capture frame-by-frame
        ret, frame = raw.read()
        if ret == True:
    
            # Display the resulting frame
            process_canny(frame)
            #process_orb(frame)
    
            # Press Q on keyboard to  exit
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break
    
        # Break the loop
        else: 
            break
    
    # When everything done, release 
    # the video capture object
    raw.release()
    
    # Closes all the frames
    cv2.destroyAllWindows()
