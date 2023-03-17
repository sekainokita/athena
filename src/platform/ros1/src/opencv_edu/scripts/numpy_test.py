#!/usr/bin/env python3

import cv2
import numpy as np
from time import *

height = 150
width = 150
img1 = np.zeros((height, width,3), np.uint8)
img2 = np.zeros((height, width), np.uint8)

img1[0:50,50:100] = [255,255,255] # BGR
img2[100:150,100:150] = 50

cv2.line(img1, (50,75), (100,80), [0,0,255], 1)
cv2.rectangle(img2, [25,125],[50,100],[255,255,255],1)
cv2.namedWindow("img1", cv2.WINDOW_NORMAL)
cv2.namedWindow("img2", cv2.WINDOW_NORMAL)
cv2.imshow("img1", img1)
cv2.imshow("img2", img2)

cv2.waitKey()
cv2.destroyAllWindows()

