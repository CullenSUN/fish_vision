#!/usr/bin/env python3

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
from opencv_apps.msg import Rect
from opencv_apps.msg import RectArray
from cv_bridge import CvBridge, CvBridgeError

WINDOW_NAME = "BREED Fish Vision"

class VisionDebugger:

    def __init__(self):
        self.bridge = CvBridge()
        self.obstacle_rects_buffer = None
        self.throttling_counter = 0
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback_image)
        rospy.loginfo("VisionDebugger subscribed to topic /raspicam_node/image/compressed")

        self.obstacles_pub = rospy.Subscriber("/obstacle_detector_node/obstacles", RectArray, self.callback_rects)
        rospy.loginfo("VisionDebugger subscribed to topic /obstacle_detector_node/obstacles")

    def increase_counter(self):
        self.throttling_counter += 1
        if self.throttling_counter >= 5:
            self.throttling_counter = 0

    def resize_image(self, img, factor):
        width = int(img.shape[1] * factor)
        height = int(img.shape[0] * factor)
        dim = (width, height)
        return cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

    def callback_image(self, data):
        if self.throttling_counter % 5 != 0:
            self.increase_counter()
            return

        try:
            img = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
            img = self.resize_image(img, 0.5)
        except CvBridgeError as e:
            rospy.logerr("error: %s", e)

        # draw obstacle rects in the image
        if self.obstacle_rects_buffer is not None: 
            for rect in self.obstacle_rects_buffer:
                #ignore very small rects
                if rect.width * rect.height < 16.0: continue 
                
                start_point = (int(rect.x), int(rect.y))
                end_point = (int(rect.x + rect.width), int(rect.y + rect.height))
                cv2.rectangle(img, start_point, end_point, (0, 255, 0), 2)

        cv2.imshow(WINDOW_NAME, img)
        cv2.waitKey(24)
        self.increase_counter()

    def callback_rects(self, rects_msg):
        #print("received obstacle rects %s" % len(rects_msg.rects))
        self.obstacle_rects_buffer = rects_msg.rects


def main(args):
    rospy.init_node('debug_node', anonymous=True)
    od = VisionDebugger()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
