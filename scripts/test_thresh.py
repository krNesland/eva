#!/usr/bin/env python

import cv2 as cv
import numpy as np

bgr = cv.imread("blue1.png")
hsv = cv.cvtColor(bgr, cv.COLOR_BGR2HSV)

lowerBlue = np.array([110,50,50])
upperBlue = np.array([130,255,255])

mask = cv.inRange(hsv, lowerBlue, upperBlue)
res = cv.bitwise_and(bgr, bgr, mask=mask)

cv.imshow('frame', bgr)
cv.imshow('mask', mask)
cv.imshow('res', res)
cv.waitKey(0)
cv.destroyAllWindows()

cv.destroyAllWindows()