#!/usr/bin/env python

"""
SUBSCRIBES TO:
    /obstacle_detector_node/obstacles: detected cordinates of obstacles

"""
import sys
import rospy
from opencv_apps.msg import RectArray

class ObstacleAvoidance:
    def __init__(self):
        self.obstacles_sub = rospy.Subscriber("/obstacle_detector_node/obstacles", RectArray, self.callback)
        rospy.loginfo("ObstacleAvoidance subscribed to topic /obstacle_detector_node/obstacles")

    def callback(self, rects_msg):
        rect_array = rects_msg.rects
        rospy.loginfo("number of rects received: %s", len(rect_array))
        if len(rect_array) > 0: 
            rospy.loginfo("first rect", rect_array[0])


def main(args):
    rospy.init_node('obstacle_avoidance_node', anonymous=True)
    ov = ObstacleAvoidance()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
   
if __name__ == '__main__':
    main(sys.argv)
