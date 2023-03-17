#!/usr/bin/env python3

import cv2
import numpy as np

img = cv2.imread("Lenna.png", cv2.IMREAD_ANYCOLOR)

cv2.namedWindow("img", cv2.WINDOW_NORMAL)
cv2.imshow("img", img)
cv2.waitKey()
cv2.destroyAllWindows()
