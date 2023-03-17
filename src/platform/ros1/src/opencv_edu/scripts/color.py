#!/usr/bin/env python3

import cv2
import numpy as np

img_bgr = cv2.imread("lane-detection-using-machine-learning.jpeg", cv2.IMREAD_ANYCOLOR)

b,g,r = cv2.split(img_bgr)

img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

h,s,v = cv2.split(img_hsv)

cv2.namedWindow("img_bgr", cv2.WINDOW_NORMAL)
cv2.namedWindow("img_hsv", cv2.WINDOW_NORMAL)
cv2.namedWindow("b", cv2.WINDOW_NORMAL)
cv2.namedWindow("g", cv2.WINDOW_NORMAL)
cv2.namedWindow("r", cv2.WINDOW_NORMAL)
cv2.namedWindow("h", cv2.WINDOW_NORMAL)
cv2.namedWindow("s", cv2.WINDOW_NORMAL)
cv2.namedWindow("v", cv2.WINDOW_NORMAL)

cv2.imshow("img_bgr", img_bgr)
cv2.imshow("img_hsv", img_hsv)
cv2.imshow("b", b)
cv2.imshow("g", g)
cv2.imshow("r", r)
cv2.imshow("h", h)
cv2.imshow("s", s)
cv2.imshow("v", v)

cv2.waitKey()
cv2.destroyAllWindows()