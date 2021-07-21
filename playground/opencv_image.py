#!/usr/bin/env python

import cv2
import numpy as np
import os
import matplotlib.pyplot as plt

def resize_image(img, factor):
    width = int(img.shape[1] * factor)
    height = int(img.shape[0] * factor)
    dim = (width, height)
    return cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

# calculate distance between two contours
def calculate_contour_distance(contour1, contour2): 
    rect1 = cv2.minAreaRect(contour1)
    box1 = np.int0(cv2.boxPoints(rect1))

    rect2 = cv2.minAreaRect(contour2)
    box2 = np.int0(cv2.boxPoints(rect2))

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

def agglomerative_cluster(contours, threshold_distance=50.0):
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
            current_contours[index1] = np.concatenate((current_contours[index1], current_contours[index2]), axis=0)
            del current_contours[index2]
        else: 
            break

    return current_contours

def detect_objects(img): 
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(imgray, 100, 200)
    ret, thresh = cv2.threshold(edges, 127, 255, 0)
    img2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)

    contours = take_biggest_contours(contours)
    contours = agglomerative_cluster(contours)

    objects = []
    for c in contours:
        rect = cv2.boundingRect(c)
        x, y, w, h = rect
        cropped_img = img[y:y+h, x:x+w].copy()
        object_tuple = (rect, cropped_img)
        objects.append(object_tuple)
    return objects

"""
Scale down current objects to match with previous objects by template. If match, it's obstacle.
"""
def detect_obstacles(previous_objects, current_objects):
    # downscale current objects to cater occlusion 
    scales = [0.85, 0.8, 0.75, 0.7, 0.65]
    obstacle_rects = set()
    for (rect2, obj2) in current_objects:
        for (rect1, obj1) in previous_objects:
            for scale in scales:
                width = int(obj2.shape[1] * scale)
                height = int(obj2.shape[0] * scale)
                scaled_obj2 = cv2.resize(obj2, (width, height), interpolation=cv2.INTER_AREA)
                if match_by_template(obj1, scaled_obj2):
                    # inverse scale to make sense of risk level
                    enlarging_scale = 1/scale
                    obstacle_rects.add((rect2, enlarging_scale)) 
                    break
    return obstacle_rects

def match_by_template(img, template, threshold_score=0.90):
    i_height, i_width, i_color = img.shape
    t_height, t_width, t_color = template.shape

    # make sure img is bigger than the template
    if i_height <= t_height or i_width <= t_width:
        return False

    result = cv2.matchTemplate(img, template, cv2.TM_CCORR_NORMED)
    _minVal, _maxVal, minLoc, maxLoc = cv2.minMaxLoc(result, None)
    if _maxVal > threshold_score:
        print("got it")
    return _maxVal > threshold_score

def match_by_features(img1, img2):
    orb = cv2.ORB_create()
    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)
    print("kp1", len(kp1))
    print("kp2", len(kp2))

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)
    # Sort them in the order of their distance.
   
    print("matches", len(matches))
    if len(matches) > 25:
        matches = sorted(matches, key = lambda x: x.distance)
        # Draw first 10 matches.
        img3 = cv2.drawMatches(img1, kp1, 
                               img2, kp2, 
                               matches[:10], None, 
                               flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        plt.imshow(img3),plt.show()

def process_by_contours(img): 
    objects = detect_objects(img)
    print("detected objects: ", len(objects))

    for (rect, cropped_img) in objects:
        x, y, w, h = rect
        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

    cv2.imshow('process_by_contours', img)

    if cv2.waitKey(0):
        cv2.destroyAllWindows()

if __name__ == '__main__':
    script_path = os.path.dirname(os.path.realpath(__file__))

    img_path1 = os.path.join(script_path, 'images/wall_1.jpg')
    img1 = cv2.imread(img_path1)
    objects1 = detect_objects(img1)
    print("detected objects1, ", len(objects1))

    img_path3 = os.path.join(script_path, 'images/wall_3.jpg')
    img3 = cv2.imread(img_path3)
    objects3 = detect_objects(img3)
    print("detected objects3, ", len(objects3))

    obstacle_rects = detect_obstacles(previous_objects=objects1, current_objects=objects3)
    print("confirmed obstacles, ", len(obstacle_rects))

    for (rect, scale) in obstacle_rects: 
        x, y, w, h = rect
        red_color = min(255, 128*scale)
        cv2.rectangle(img3, (x, y), (x+w, y+h), (0, 0, red_color), 2)
    
    cv2.imshow("Obstacles in Image", img3)

    if cv2.waitKey(0):
        cv2.destroyAllWindows()
