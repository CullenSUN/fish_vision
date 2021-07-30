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
    x1, y1, w1, h1 = cv2.boundingRect(contour1)
    c_x1 = x1 + w1/2
    c_y1 = y1 + h1/2

    x2, y2, w2, h2 = cv2.boundingRect(contour2)
    c_x2 = x2 + w2/2
    c_y2 = y2 + h2/2

    return max(abs(c_x1 - c_x2) - (w1 + w2)/2, abs(c_y1 - c_y2) - (h1 + h2)/2)

# only keep the biggest tetake_biggest_contoursn to make computation faster 
def take_biggest_contours(contours, max_number=10):
    sorted_contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
    return sorted_contours[:max_number]

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

def detect_objects(img): 
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    equalized_img = cv2.equalizeHist(gray_img)
    blurred_img = cv2.GaussianBlur(equalized_img,(9,9),0)
    edges = cv2.Canny(blurred_img, 100, 200)
    ret, thresh = cv2.threshold(edges, 127, 255, 0)
    # saliency = cv2.saliency.StaticSaliencyFineGrained_create()
    # success, saliencyMap = saliency.computeSaliency(blurred_img)
    # ret, thresh = cv2.threshold(saliencyMap.astype("uint8"), 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    img2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)

    contours = take_biggest_contours(contours)
    contours = agglomerative_cluster(contours)

    objects = []
    for c in contours:
        rect = cv2.boundingRect(c)
        x, y, w, h = rect
        
        # ignore small contours
        if w < 10 or h < 10:
            continue

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
