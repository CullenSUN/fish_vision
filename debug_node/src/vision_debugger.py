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
from cv_bridge import CvBridge, CvBridgeError

WINDOW_NAME = "BREED Fish Vision"

class VisionDebugger:

    def __init__(self):
        self.bridge = CvBridge()
        self.obstacle_rects_buffer = None
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback_image)
        print("VisionDebugger subscribed to topic /raspicam_node/image/compressed")

        self.obstacles_pub = rospy.Subscriber("/obstacle_detector_node/obstacles", RectArray, self.callback_rects)
        print("VisionDebugger subscribed to topic /obstacle_detector_node/obstacles")

    def callback_image(self, data):
        print("received image")
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # draw obstacle rects in the image
        if self.obstacle_rects_buffer is not None: 
            for rect in self.obstacle_rects_buffer :
                x, y, w, h = rect
                cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

        cv2.imshow(WINDOW_NAME, img)
        cv2.waitKey(24)

    def callback_rects(self, rects_msg):
        print("received obstacle rects %s" % len(rects_msg.rects))
        self.obstacle_rects_buffer = rects_msg.rects


def main(args):
    od = VisionDebugger()
    rospy.init_node('debug_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
