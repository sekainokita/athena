#!/usr/bin/env python3

import cv2
import numpy as np

img_bgr = cv2.imread("lane-detection-using-machine-learning.jpeg", cv2.IMREAD_ANYCOLOR)
img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

# H,S,V
lower_y = np.uint8([0,80,200])
upper_y = np.uint8([45,255,255])
mask_y = cv2.inRange(img_hsv, lower_y, upper_y)

lower_w = np.uint8([0,0,200])
upper_w = np.uint8([179,25,255])
mask_w = cv2.inRange(img_hsv, lower_w, upper_w)

masked_y = cv2.bitwise_and(img_bgr, img_bgr, mask=mask_y)
bitwise_img = cv2.bitwise_or(mask_y, mask_w)
masked_all = cv2.bitwise_and(img_bgr, img_bgr, mask=bitwise_img)

cv2.imshow("img_bgr", img_bgr)
cv2.imshow("mask_w", mask_w)
cv2.imshow("mask_y", mask_y)
cv2.imshow("img_bitwise", masked_y)
cv2.imshow("bitwise_img", bitwise_img)
cv2.imshow("masked_all", masked_all)

cv2.waitKey()
cv2.destroyAllWindows()