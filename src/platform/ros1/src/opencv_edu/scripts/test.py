#!/usr/bin/env python3

import cv2
import numpy as np

blue    = [ 255,    0,      0   ]
green   = [ 0,      255,    0   ]
red     = [ 0,      0,      255 ]
white   = [ 255,    255,    0   ]
black   = [ 0,      0,      0   ]
gray    = [ 127,    127,    127 ]
cyan    = [ 255,    255,    0   ]
magenta = [ 255,    0,      255 ]
yellow  = [ 0,      255,    255 ]

color = yellow


img = np.uint8(
            [  
               [color,color,color,color,color],
               [color,color,color,color,color],
               [color,color,color,color,color],
               [color,color,color,color,color],
               [color,color,color,color,color],
            ]
)

cv2.imshow("asd", img)
cv2.waitKey()
cv2.destroyAllWindows()