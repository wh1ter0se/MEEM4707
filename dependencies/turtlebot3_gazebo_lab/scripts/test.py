#!/usr/bin/env python
# license removed for brevity

import math
import numpy as np
import sys
import cv2
#from cv_bridge import CvBridge, CvBridgeError

img = cv2.imread("test.png")

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
lower_range = np.array([0,0,150])
upper_range = np.array([255,255,255])

mask = cv2.inRange(hsv, lower_range, upper_range)

cv2.imshow('image', img)
cv2.imshow('mask', mask)

while(True):
	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break

cv2.destroyAllWindows()