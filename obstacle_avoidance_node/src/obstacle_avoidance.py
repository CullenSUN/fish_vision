#!/usr/bin/env python

"""
SUBSCRIBES TO:
    /obstacle_detector_node/obstacles: detected cordinates of obstacles

"""

import rospy
from opencv_apps.msg import RectArray


class ObstacleAvoidance:
    def __init__(self):
        self.obstacles_sub = rospy.Subscriber("/obstacle_detector_node/obstacles")


 
