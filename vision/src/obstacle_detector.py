#!/usr/bin/env python

"""
SUBSCRIBES TO:
    /raspicam_node/image: Source image topic
    
PUBLISHES TO:
    /vision/obstacles: detected postion of obstacles

"""

import sys
import cv2
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ObstacleDetector:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/raspicam_node/image", Image, self.callback)
        print("Subscribed to topic /raspicam_node/image")

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)

        print("image shape is", cv_image.shape)
        # (rows, cols, channels) = cv_image.shape


def main(args):
    od = ObstacleDetector()
    rospy.init_node('obstacle_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    

if __name__ == '__main__':
    main(sys.argv)