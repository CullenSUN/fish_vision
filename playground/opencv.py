import numpy as np
import cv2

img = cv2.imread('test_image_1.jpeg')
imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(imgray, 127, 255, 0)
img2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

print("found: ", len(contours))

cv2.drawContours(img, contours, -1, (0,255,0), 3)

cv2.namedWindow('Contours',cv2.WINDOW_NORMAL)
cv2.imshow('Contours', img)

if cv2.waitKey(0):
    cv2.destroyAllWindows()