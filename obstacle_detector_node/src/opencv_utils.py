#!/usr/bin/env python

import cv2
import numpy
from functools import wraps
import time

def timed(f):
    @wraps(f)
    def wrapper(*args, **kwds):
        start = time()
        result = f(*args, **kwds)
        elapsed = time() - start
        print "%s took %d time to finish" % (f.__name__, elapsed)
        return result
    return wrapper

def resize_image(img, factor):
    width = int(img.shape[1] * factor)
    height = int(img.shape[0] * factor)
    dim = (width, height)
    return cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

# calculate distance between two contours
def calculate_contour_distance(contour1, contour2): 
    x1, y1, w1, h1 = cv2.boundingRect(contour1)
    c_x1 = x1 + w1/2
    c_y1 = y1 + h1/2

    x2, y2, w2, h2 = cv2.boundingRect(contour2)
    c_x2 = x2 + w2/2
    c_y2 = y2 + h2/2

    return max(abs(c_x1 - c_x2) - (w1 + w2)/2, abs(c_y1 - c_y2) - (h1 + h2)/2)

def take_biggest_contours(contours, max_number=20):
    sorted_contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
    return sorted_contours[:max_number]

def agglomerative_cluster(contours, threshold_distance=40.0):
    start = time.time()
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
    elapsed = (time.time() - start)
    print("dddddd agglomerative_cluster() took ", elapsed, " time to finish")
    return current_contours

@timed
def detect_objects(img): 
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    equalized_img = cv2.equalizeHist(gray_img)
    blurred_img = cv2.GaussianBlur(equalized_img, (9,9), 0)
    edges = cv2.Canny(blurred_img, 90, 180)
    ret, thresh = cv2.threshold(edges, 127, 255, 0)

    img2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
    print("contours after findContours: %s" % len(contours))

    contours = take_biggest_contours(contours)
    print("contours after take_biggest_contours: %s" % len(contours))

    contours = agglomerative_cluster(contours)
    print("contours after agglomerative_cluster: %s" % len(contours))

    objects = []
    for c in contours:
        rect = cv2.boundingRect(c)
        x, y, w, h = rect
        
        # ignore small contours
        if w < 20 and h < 20:
            print("dropping rect due to small size", rect)
            continue

        cropped_img = img[y:y+h, x:x+w].copy()
        object_tuple = (rect, cropped_img)
        objects.append(object_tuple)
    return objects

"""
Scale down current objects to match with previous objects by template. If match, it's obstacle.
"""
@timed
def detect_obstacles(previous_frame, current_objects):
    # downscale current objects to cater occlusion 
    scales = [0.9, 0.8, 0.7, 0.6, 0.5]
    obstacle_rects = set()
    for (rect2, obj2) in current_objects:
        for scale in scales:
            width = int(obj2.shape[1] * scale)
            height = int(obj2.shape[0] * scale)
            scaled_obj2 = cv2.resize(obj2, (width, height), interpolation=cv2.INTER_AREA)
            if match_by_template(previous_frame, scaled_obj2):
                # inverse scale to make sense of risk level
                enlarging_scale = 1/scale
                obstacle_rects.add((rect2, enlarging_scale)) 
                break
    return obstacle_rects

"""
Check if we can find a match for the template in the img. Return True if found, else return False.
"""
@timed
def match_by_template(img, template, threshold_score=0.95):
    i_height, i_width, i_color = img.shape
    t_height, t_width, t_color = template.shape

    # make sure img is bigger than the template
    if i_height <= t_height or i_width <= t_width:
        return False

    result = cv2.matchTemplate(img, template, cv2.TM_CCORR_NORMED)
    _minVal, _maxVal, minLoc, maxLoc = cv2.minMaxLoc(result, None)
    return _maxVal > threshold_score