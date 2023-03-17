#!/usr/bin/env python3

import cv2
import numpy as np

play_video = cv2.VideoCapture("lane_video.mp4")
lower_y = np.uint8([0,80,100])
upper_y = np.uint8([45,255,255])

lower_w = np.uint8([0,0,150])
upper_w = np.uint8([179,25,255])

while True:
    ret, img_bgr = play_video.read()
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    mask_y = cv2.inRange(img_hsv, lower_y, upper_y)
    mask_w = cv2.inRange(img_hsv, lower_w, upper_w)
    masked_y = cv2.bitwise_and(img_bgr, img_bgr, mask=mask_y)
    bitwise_img = cv2.bitwise_or(mask_y, mask_w)
    masked_all = cv2.bitwise_and(img_bgr, img_bgr, mask=bitwise_img)

    # H,S,V
    # print(masked_all.shape)

    img_y,img_x,_ = masked_all.shape # _ means (it will be disapeared)
    ref_x = round(img_x * 0.22) # (960-825)/960
    center_ref_x = round(img_x * 0.40) # 400 (left yellow) - center - 590 (right white)
                                            # 960-590=>385/960=0.40, 370+400/20=38.5
    center_ref_y = round(img_y * 0.66)
                                            # 360/540 = 0.666666667
    src_point1 = [ref_x, img_y]
    src_point2 = [center_ref_x, center_ref_y]
    src_point3 = [img_x - center_ref_x, center_ref_y]
    src_point4 = [img_x-ref_x, img_y]

    src_points = np.float32(
        [
            [src_point1],
            [src_point2],
            [src_point3],
            [src_point4],
        ]
    )

    size = 1000
    dst_ref_x = 100
    dst_point1 = [dst_ref_x, size]
    dst_point2 = [dst_ref_x,0]
    dst_point3 = [size - dst_ref_x, 0]
    dst_point4 = [size - dst_ref_x, size]

    dst_points = np.float32(
        [
            [dst_point1],
            [dst_point2],
            [dst_point3],
            [dst_point4],
        ]
    )

    matrix = cv2.getPerspectiveTransform(src_points, dst_points)
    warp_img = cv2.warpPerspective(masked_all, matrix, [size, size])

    cv2.line(masked_all, src_point1, src_point1, [255,0,0], 10)
    cv2.line(masked_all, src_point2, src_point2, [0,255,0], 10)
    cv2.line(masked_all, src_point3, src_point3, [0,0,255], 10)
    cv2.line(masked_all, src_point4, src_point4, [0,255,255], 10)

    # left_x 160
    # right_x 960-870 = 110

    cv2.imshow("img_bgr", img_bgr)
    cv2.imshow("mask_w", mask_w)
    cv2.imshow("mask_y", mask_y)
    cv2.imshow("img_bitwise", masked_y)
    cv2.imshow("bitwise_img", bitwise_img)
    cv2.imshow("masked_all", masked_all)
    cv2.imshow("warp_img", warp_img)
    a = cv2.waitKey(1)
    if a != -1:
        cv2.waitKey(0)
    else:
        pass

cv2.waitKey()
cv2.destroyAllWindows()