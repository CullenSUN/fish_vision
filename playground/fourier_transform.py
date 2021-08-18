import os
import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

script_path = os.path.dirname(os.path.realpath(__file__))
img_path = os.path.join(script_path, 'images/pool_image_copy.png')
img = cv.imread(img_path, 0)
dft = cv.dft(np.float32(img),flags = cv.DFT_COMPLEX_OUTPUT)
dft_shift = np.fft.fftshift(dft)
magnitude_spectrum = 20*np.log(cv.magnitude(dft_shift[:,:,0],dft_shift[:,:,1]))

rows, cols = img.shape
crow, ccol = rows/2, cols/2
# create a mask first, center square is 1, remaining all zeros
mask = np.zeros((rows,cols,2), np.uint8)
mask[crow-30:crow+30, ccol-30:ccol+30] = 1
# apply mask and inverse DFT
fshift = dft_shift*mask
f_ishift = np.fft.ifftshift(fshift)
img_back = cv.idft(f_ishift)
img_back = cv.magnitude(img_back[:,:,0], img_back[:,:,1])

output_img_path = os.path.join(script_path, 'images/ouput.png')
cv.imwrite(output_img_path, img_back)

plt.subplot(121),plt.imshow(img, cmap = 'gray')
plt.title('Input Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(img_back, cmap = 'gray')
plt.title('Magnitude Spectrum'), plt.xticks([]), plt.yticks([])

plt.show()
