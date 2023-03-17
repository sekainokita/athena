#!/usr/bin/env python3

import cv2
import numpy as np
from time import *

cap = cv2.VideoCapture("file_example_MP4_1920_18MG.mp4")
fourcc = cv2.VideoWriter_fourcc(*"XVID")
out = cv2.VideoWriter("video_save.avi", fourcc, 30.0, [1920,1080])

start_time = time()

while True:
    second_time = time()
    ret, img = cap.read()
    duration = second_time - start_time
    cv2.imshow("video", img)
    key = cv2.waitKey(1)

    if key == ord("s"):
        out.write(img)
    elif key == ord("q"):
        cap.release()
        break

cv2.waitKey()
cv2.destroyAllWindows()

