#!/usr/bin/env python

"""
SUBSCRIBES TO:
    /raspicam_node/image: Source image topic
    /obstacle_detector_node/obstacles: detected cordinates of obstacles

PUBLISHES TO:
    n/a

"""

import sys
import cv2
import rospy

from sensor_msgs.msg import CompressedImage
from opencv_apps.msg import RectArray
#from opencv_apps.msg import Rect
from cv_bridge import CvBridge, CvBridgeError

def rect_contains(rect1, rect2):
    x1, y1, w1, h1 = rect1
    x2, y2, w2, h2 = rect2
    return x1 <= x2 and y1 <= y2 and x1+w1 >= x2+w2 and y1+h1 >= y2+h2 

class VisionDebugger:

    WINDOW_NAME = "BREED Fish Vision"

    def __init__(self):
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        self.bridge = CvBridge()
        self.image_buffer = None
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback_image)
        print("VisionDebugger subscribed to topic /raspicam_node/image/compressed")

        self.obstacles_pub = rospy.Subscriber("/obstacle_detector_node/obstacles", RectArray, self.callback_rects)
        print("VisionDebugger subscribed to topic /obstacle_detector_node/obstacles")
        
    def __del__(self):
        print('VisionDebugger destructor called, Employee deleted.')
        cv2.destroyAllWindows()

    def callback_image(self, data):
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(data)
            cv2.imshow(WINDOW_NAME, img)
            self.image_buffer = img
        except CvBridgeError as e:
            print(e)

        # print("image shape is", cv_image.shape)

    def callback_rects(self, rects_msg):
        if self.image_buffer is None: return

        img = image_buffer
        rect_array = rects_msg.rects
        print("number of rects received: %s" % len(rect_array))

        for rect in rect_array:
            x, y, w, h = rect
            cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

        cv2.imshow(WINDOW_NAME, img)


def main(args):
    od = VisionDebugger()
    rospy.init_node('debug_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    

if __name__ == '__main__':
    main(sys.argv)
