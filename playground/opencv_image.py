#!/usr/bin/env python

import cv2
import numpy as np
import os
import matplotlib.pyplot as plt
import random as rng
from functools import wraps
from time import time

def timed(f):
    @wraps(f)
    def wrapper(*args, **kwds):
        start = time()
        result = f(*args, **kwds)
        elapsed = time() - start
        print "%s took %d milliSeconds to finish" % (f.__name__, elapsed*1000)
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

# only keep the biggest tetake_biggest_contoursn to make computation faster 
def take_biggest_contours(contours, max_number=20):
    sorted_contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
    return sorted_contours[:max_number]

@timed
def agglomerative_cluster(contours, threshold_distance=40.0):
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

@timed
def detect_objects(img, fgMask=None): 
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if fgMask is None:
        equalized_img = cv2.equalizeHist(gray_img)
        blurred_img = cv2.GaussianBlur(equalized_img, (9,9), 0)
        edges = cv2.Canny(blurred_img, 90, 180)
        ret, thresh = cv2.threshold(edges, 127, 255, 0)
    else:
        blurred_img = cv2.GaussianBlur(fgMask, (9,9), 0)
        ret, thresh = cv2.threshold(blurred_img, 127, 255, 0)

    # saliency = cv2.saliency.StaticSaliencyFineGrained_create()
    # success, saliencyMap = saliency.computeSaliency(blurred_img)
    # ret, thresh = cv2.threshold(saliencyMap.astype("uint8"), 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
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

def show_contours(img):
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    equalized_img = cv2.equalizeHist(gray_img)
    blurred_img = cv2.GaussianBlur(equalized_img, (9,9), 0)
    edges = cv2.Canny(blurred_img, 90, 180)
    ret, thresh = cv2.threshold(edges, 127, 255, 0)
    img2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)

    drawing = np.zeros((edges.shape[0], edges.shape[1], 3), dtype=np.uint8)
    for i in range(len(contours)):
        #color = (255, 255, 255)
        color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
        cv2.drawContours(drawing, contours, i, color, 2, cv2.LINE_8, hierarchy, 0)

    print("found %s contours " % len(contours))
    # Show in a window
    cv2.imshow('Contours', drawing)
    if cv2.waitKey(0):
        cv2.destroyAllWindows()

def show_contours_with_original_image(img): 
    objects = detect_objects(img)
    print("detected objects: ", len(objects))

    for (rect, cropped_img) in objects:
        x, y, w, h = rect
        cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)

    cv2.imshow('process_by_contours', img)

    if cv2.waitKey(0):
        cv2.destroyAllWindows()

"""
Scale down current objects to match with previous objects by template. If match, it's obstacle.
"""
@timed
def detect_obstacles(previous_frame, current_objects):
    # downscale current objects to cater occlusion 
    scales = [0.8, 0.7, 0.6]
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

@timed
def match_by_template(img, template, threshold_score=0.95):
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray_template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    i_height, i_width = gray_img.shape
    t_height, t_width = gray_template.shape

    # make sure img is bigger than the template
    if i_height <= t_height or i_width <= t_width:
        return False

    result = cv2.matchTemplate(gray_img, gray_template, cv2.TM_CCORR_NORMED)
    _minVal, _maxVal, minLoc, maxLoc = cv2.minMaxLoc(result, None)
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

def low_pass_filter(img, frequence_threshold = 50):
    dft = cv2.dft(np.float32(img), flags = cv2.DFT_COMPLEX_OUTPUT)
    dft_shift = np.fft.fftshift(dft)
    rows, cols = img.shape
    crow, ccol = rows/2, cols/2
    # create a mask first, center square is 1, remaining all zeros
    mask = np.zeros((rows,cols,2), np.uint8)
    mask[crow-frequence_threshold:crow+frequence_threshold, ccol-frequence_threshold:ccol+frequence_threshold] = 1

    # apply mask and inverse DFT
    fshift = dft_shift * mask
    f_ishift = np.fft.ifftshift(fshift)
    img_back = cv2.idft(f_ishift, flags = cv2.DFT_SCALE | cv2.DFT_REAL_OUTPUT)
    img_back = np.uint8(img_back) 
    return img_back

if __name__ == '__main__':
    print("Numpy version: %s" % np.__version__)
    print("OpenCV version: %s" % cv2.__version__)
    script_path = os.path.dirname(os.path.realpath(__file__))

    img_path1 = os.path.join(script_path, 'images/wall_1.jpg')
    img1 = cv2.imread(img_path1)

    # objects1 = detect_objects(img1)
    # print("detected objects1, ", len(objects1))

    img_path2 = os.path.join(script_path, 'images/wall_4.jpg')
    img2 = cv2.imread(img_path2)
    objects3 = detect_objects(img2)
    print("detected objects3, ", len(objects3))

    obstacle_rects = detect_obstacles(previous_frame=img1, current_objects=objects3)
    print("confirmed obstacles, ", len(obstacle_rects))

    for (rect, scale) in obstacle_rects: 
        x, y, w, h = rect
        red_color = min(255, 128*scale)
        cv2.rectangle(img2, (x, y), (x+w, y+h), (0, 0, red_color), 2)
    
    cv2.imshow("Obstacles in Image", img2)

    if cv2.waitKey(0):
        cv2.destroyAllWindows()
