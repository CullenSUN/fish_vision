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
from cv_bridge import CvBridge, CvBridgeError

def rect_contains(rect1, rect2):
    x1, y1, w1, h1 = rect1
    x2, y2, w2, h2 = rect2
    return x1 <= x2 and y1 <= y2 and x1+w1 >= x2+w2 and y1+h1 >= y2+h2 

class ObstacleDetector:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.callback)
        print("ObstacleDetector subscribed to topic /raspicam_node/image/compressed")
        self.obstacles_pub = rospy.Publisher("/obstacle_detector_node/obstacles", RectArray, queue_size=5)

    def callback(self, data):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data)

        except CvBridgeError as e:
            print(e)

        # print("image shape is", cv_image.shape)

    def process_image(self, img): 
        imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(imgray, 100, 200)
        ret, thresh = cv2.threshold(edges, 127, 255, 0)
        img2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # find bounding_rects
        bounding_rects = []
        for c in contours:
            rect = cv2.boundingRect(c)
            bounding_rects.append(rect)

        # ignore those rects that are completely contained by other bigger rects
        valid_rects = []
        for i, rect_i in enumerate(bounding_rects):
            is_contained = False
            for j, rect_j in enumerate(bounding_rects):
                if j == i: continue
                if rect_contains(rect_j, rect_i):
                    is_contained = True
                    break

            if not is_contained: 
                valid_rects.append(rect_i)

        self.obstacles_pub.publish(valid_rects)


def main(args):
    od = ObstacleDetector()
    rospy.init_node('obstacle_detector_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    

if __name__ == '__main__':
    main(sys.argv)
