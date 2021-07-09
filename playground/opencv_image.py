#!/usr/bin/env python

import cv2
import numpy
import os

# calculate distance between two contours
def calculate_contour_distance(contour1, contour2): 
    center1, radius1 = cv2.minEnclosingCircle(contour1)
    center2, radius2 = cv2.minEnclosingCircle(contour2)
    return cv2.norm(center1, center2) - (radius1 + radius2)

# only keep the biggest ten to make computation faster 
def take_biggest_contours(contours, max_number=10):
    sorted_contours = sorted(contours, key=lambda x: cv2.minEnclosingCircle(x)[1], reverse=True)
    return sorted_contours[:max_number]

def agglomerative_cluster(contours, threshold_distance=10.0):
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

def process_canny(img): 
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(imgray, 100, 200)
    ret, thresh = cv2.threshold(edges, 127, 255, 0)
    img2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
    print("found contours: ", len(contours))

    contours = take_biggest_contours(contours)
    print("keep biggest ten contours. remaining: ", len(contours))

    contours = agglomerative_cluster(contours)
    print("clustered contours: ", len(contours))

    for c in contours:
        # get the bounding rect
        x, y, w, h = cv2.boundingRect(c)
        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

    #cv2.drawContours(img, contours, -1, (0, 0, 255), 3)
    cv2.imshow('Contours', img)

# to detect the corners
def process_orb(img):
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Initiate ORB detector
    orb = cv2.ORB_create()
    # find the keypoints with ORB
    kp = orb.detect(imgray, None)

    # compute the descriptors with ORB
    kp, des = orb.compute(imgray, kp)
    print("keypoints", kp)
    # draw only keypoints location,not size and orientation
    img2 = cv2.drawKeypoints(img, kp, None, color=(0, 255, 0), flags=0)
    cv2.imshow('Contours_orb', img2)


if __name__ == '__main__':
    cv2.namedWindow('Contours_canny', cv2.WINDOW_NORMAL)
    script_path = os.path.dirname(os.path.realpath(__file__))
    img_path = os.path.join(script_path, 'images/test_image_2.jpeg')
    print(img_path)
    img = cv2.imread(img_path)
    process_canny(img)
    #process_orb(img)

    if cv2.waitKey(0):
        cv2.destroyAllWindows()

