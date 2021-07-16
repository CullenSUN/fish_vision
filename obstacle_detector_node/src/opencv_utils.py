#!/usr/bin/env python

import cv2
import numpy

def resize_image(img, factor):
    width = int(img.shape[1] * factor)
    height = int(img.shape[0] * factor)
    dim = (width, height)
    return cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

def contains(rect1, rect2):
    x1, y1, w1, h1 = rect1
    x2, y2, w2, h2 = rect2
    return x1 <= x2 and y1 <= y2 and x1+w1 >= x2+w2 and y1+h1 >= y2+h2 

# calculate distance between two contours
def calculate_contour_distance(contour1, contour2): 
    rect1 = cv2.minAreaRect(contour1)
    box1 = numpy.int0(cv2.boxPoints(rect1))

    rect2 = cv2.minAreaRect(contour2)
    box2 = numpy.int0(cv2.boxPoints(rect2))

    distances = []
    for point1 in box1:
        for point2 in box2:
            distance = cv2.norm(point1, point2)
            distances.append(distance)

    return min(distances)

def calculate_contour_area(contour):
  _, (w, h), _ = cv2.minAreaRect(contour)
  return w * h

# only keep the biggest ten to make computation faster 
def take_biggest_contours(contours, max_number=20):
    sorted_contours = sorted(contours, key=lambda x: calculate_contour_area(x), reverse=True)
    return sorted_contours[:max_number]

def agglomerative_cluster(contours, threshold_distance=20.0):
    current_contours = contours
    while len(current_contours) > 1:
        min_distance = None
        min_coordinate = None

        for x in xrange(len(current_contours)-1):
            for y in xrange(x+1, len(current_contours)):
                distance = calculate_contour_distance(current_contours[x], current_contours[y])
                if min_distance is None:
                    min_distance = distance
                    min_coordinate = (x, y)
                elif distance < min_distance:
                    min_distance = distance
                    min_coordinate = (x, y)

        if min_distance < threshold_distance:
            # merge closest two contours
            index1, index2 = min_coordinate
            current_contours[index1] = numpy.concatenate((current_contours[index1], current_contours[index2]), axis=0)
            del current_contours[index2]
        else: 
            break

    return current_contours
