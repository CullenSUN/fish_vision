#!/usr/bin/env python

"""
SUBSCRIBES TO:
    /raspicam_node/image: Source image topic
    
PUBLISHES TO:
    /obstacle_detector_node/obstacles: detected cordinates of obstacles

"""

import sys
import cv2
import rospy

from sensor_msgs.msg import CompressedImage
from opencv_apps.msg import RectArray
from opencv_apps.msg import Rect
from cv_bridge import CvBridge, CvBridgeError
from opencv_utils import *

class ObstacleDetector:

    def __init__(self):
        self.bridge = CvBridge()
        self.obstacles_pub = rospy.Publisher("/obstacle_detector_node/obstacles", RectArray, queue_size=5)
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback)
        rospy.loginfo("ObstacleDetector subscribed to topic /raspicam_node/image/compressed")

    def callback(self, data):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
            self.process_image(cv_image)
        except CvBridgeError as e:
            rospy.logerr("error: %s", e)

    def process_image(self, img): 
        imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(imgray, 100, 200)
        ret, thresh = cv2.threshold(edges, 127, 255, 0)
        img2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rospy.loginfo("found contours: %s", len(contours))

        contours = take_biggest_contours(contours, max_number=10)
        rospy.loginfo("keep biggest ten contours. remaining: %s", len(contours))

        contours = agglomerative_cluster(contours, threshold_distance=10.0)
        rospy.loginfo("clustered contours: %s", len(contours))

        # find bounding_rects
        rects_msg = RectArray()
        for c in contours:
            rect = cv2.boundingRect(c)
            msg_rect = Rect(*rect)
            rects_msg.rects.append(msg_rect)

        # print("number of rects published: %s" % len(rects_msg.rects))
        self.obstacles_pub.publish(rects_msg)

def main(args):
    rospy.init_node('obstacle_detector_node', anonymous=True)
    od = ObstacleDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
if __name__ == '__main__':
    main(sys.argv)
