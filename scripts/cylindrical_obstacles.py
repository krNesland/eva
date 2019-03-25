#!/usr/bin/env python

import roslib
import rospy
from eva_a.msg import *
import numpy as np
import cv2 as cv
import math

# Problem is that with small circles, the votes are more condensed and the small radius wins.
def hough_circles(img):
    maxR = 0.8*20
    minR = 0.1*20

    numR = 20
    numY, numX = img.shape # Same shape of accumulator as image (except in the radius dimension).
    numTheta = 100
    
    nonZeroRow, nonZeroCol = np.nonzero(img)

    acc = np.zeros((numY, numX, numR), dtype = np.uint32)

    for y in nonZeroRow:
        for x in nonZeroCol:
            for rCounter, r in enumerate(np.linspace(minR, maxR, numR)):
                for theta in np.linspace(-math.pi, math.pi, numTheta):
                    a = int(round(x - r*math.cos(theta))) # y coordinate of center vote.
                    b = int(round(y - r*math.sin(theta))) # x coordinate of center vote.

                    # Only considering circles with center inside the image.
                    if a >= 0 and a < numX and b >= 0 and b < numY:
                        acc[b][a][rCounter] = acc[b][a][rCounter] + math.floor(r)

    best = int(np.argmax(acc))

    bestY = best/(numX*numR)
    best = best - bestY*numX*numR
    bestX = best/numR
    best = best - bestX*numR
    bestR = best

    print(bestY, bestX, bestR)
    print(acc[bestY][bestX][bestR])

    bestRadius = int(minR + bestR*((maxR - minR)/numR))

    cv.circle(img, (bestX, bestY), bestRadius, 255)
    cv.imshow("img", img)
    cv.waitKey(3)

def callback(data):
    imgData = np.array(data.data, dtype=np.uint8)
    img = np.reshape(imgData, (data.height, data.width))

    hough_circles(img)

def listener():
    rospy.init_node('cylindrical_obstacles', anonymous=True)

    rospy.Subscriber('short_scan', ShortData, callback, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()