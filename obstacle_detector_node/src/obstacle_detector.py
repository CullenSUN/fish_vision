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
import opencv_utils

SAMPLING_PERIOD = 1

class ObstacleDetector:

    def __init__(self):
        self.bridge = CvBridge()
        self.throttling_counter = 0
        self.obstacles_pub = rospy.Publisher("/obstacle_detector_node/obstacles", RectArray, queue_size=5)
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback)
        rospy.loginfo("ObstacleDetector subscribed to topic /raspicam_node/image/compressed")
        
    def increase_counter(self):
        self.throttling_counter += 1
        if self.throttling_counter >= SAMPLING_PERIOD:
            self.throttling_counter = 0

    def callback(self, data):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data)
            self.process_image(cv_image)
        except CvBridgeError as e:
            rospy.logerr("error: %s", e)

    def process_image(self, img): 
        detected_obstacles = self.detect_obstacles(img)
        if detected_obstacles is None:
            return 

        # find bounding_rects
        rects_msg = RectArray()
        for rect in detected_obstacles or []: 
            msg_rect = Rect(*rect)
            rects_msg.rects.append(msg_rect)

        rospy.loginfo("number of rects published: %s" % len(rects_msg.rects))
        self.obstacles_pub.publish(rects_msg)

    def detect_obstacles(self, img): 
        if self.throttling_counter % SAMPLING_PERIOD != 0:
            self.increase_counter()
            return None

        # resize from 1280x720 to 640x360
        img = opencv_utils.resize_image(img, 0.5)
        current_objects = opencv_utils.detect_objects(img)
        return current_objects

def main(args):
    rospy.init_node('obstacle_detector_node', anonymous=True)
    od = ObstacleDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
if __name__ == '__main__':
    main(sys.argv)
