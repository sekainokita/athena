#!/usr/bin/env python3

import cv2
import numpy as np

img = cv2.imread("Lenna.png", cv2.IMREAD_GRAYSCALE)

cv2.namedWindow("img", cv2.WINDOW_NORMAL)
cv2.imshow("img", img)

key = cv2.waitKey(0)
if key == ord("s"):
    cv2.imwrite("Lenna_gray.png", img)
else:
    print("key:", key)

cv2.waitKey()
cv2.destroyAllWindows()

