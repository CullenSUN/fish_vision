import os
import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

script_path = os.path.dirname(os.path.realpath(__file__))
img_path = os.path.join(script_path, 'images/pool_image.png')
img = cv.imread(img_path)
blur = cv.blur(img,(5,5))
plt.subplot(121),plt.imshow(img),plt.title('Original')
plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(blur),plt.title('Averaging')
plt.xticks([]), plt.yticks([])
plt.show()