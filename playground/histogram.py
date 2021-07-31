import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
import os

script_path = os.path.dirname(os.path.realpath(__file__))
img_path = os.path.join(script_path, 'images/equalized_img.png')
img = cv.imread(img_path, 0)
plt.hist(img.ravel(), 256, [0,256])
plt.show()