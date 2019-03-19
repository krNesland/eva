#!/usr/bin/env python

import cv2 as cv
import numpy as np
import math

try:
    bgr = cv.imread("blue1.png")
except:
    print("Not able to load image.")

hsv = cv.cvtColor(bgr, cv.COLOR_BGR2HSV)

lowerBlue = np.array([110,50,50])
upperBlue = np.array([130,255,255])

mask = cv.inRange(hsv, lowerBlue, upperBlue)
res = cv.bitwise_and(bgr, bgr, mask=mask)

xMoment = 0
yMoment = 0
total = 0

for row in range(0, bgr.shape[0]):
    for col in range(0, bgr.shape[1]):
        if mask[row][col]:
            xMoment = xMoment + col
            yMoment = yMoment + row
            total = total + 1

if not total == 0:
    xArm = xMoment/total
    yArm = yMoment/total
else:
    xArm = (bgr.shape[0])/2
    yArm = (bgr.shape[1])/2

xArm = int(math.floor(xArm))
yArm = int(math.floor(yArm))

cv.drawMarker(res, (xArm, yArm), (0, 0, 255))

cv.imshow('frame', bgr)
cv.imshow('mask', mask)
cv.imshow('res', res)
cv.waitKey(0)
cv.destroyAllWindows()
