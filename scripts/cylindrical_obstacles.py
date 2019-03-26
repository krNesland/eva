#!/usr/bin/env python

import roslib
import rospy
from eva_a.msg import *
import numpy as np
import cv2 as cv
import math

# Problem is that with small circles, the votes are more condensed and the small radius wins.
def hough_circles(img):
    maxR = 0.7*20
    minR = 0.3*20

    numR = 10
    numY, numX = img.shape # Same shape of accumulator as image (except in the radius dimension).
    numTheta = 360/10
    
    nonZeroRow, nonZeroCol = np.nonzero(img)
    rows = list(nonZeroRow)
    cols = list(nonZeroCol)
    
    acc = np.zeros((numY, numX, numR), dtype = np.float)

    # (x - a)^2 + (y - b)^2 = r^2

    for rCounter, r in enumerate(np.linspace(minR, maxR, numR)):
        for y, x in zip(nonZeroRow, nonZeroCol):
            oldCenters = []
            for theta in np.linspace(-math.pi, math.pi, numTheta):
                a = int(round(x - r*math.cos(theta))) # y coordinate of center vote.
                b = int(round(y - r*math.sin(theta))) # x coordinate of center vote.

                if not (b, a) in oldCenters: # Don't want a single pixel to be able to vote on the same (a, b) more than once.
                    # Only considering circles with center inside the image.
                    if a >= 0 and a < numX and b >= 0 and b < numY:
                        acc[b][a][rCounter] = acc[b][a][rCounter] + 1

                    oldCenters.append((b, a))

    if not np.max(acc) < 1:
        best = int(np.argmax(acc)) # Index of the max in a flattened version of the array.

        # Converting to 3D indices.
        bestY = best/(numX*numR)
        best = best - bestY*numX*numR
        bestX = best/numR
        best = best - bestX*numR
        bestR = best

        print(bestY, bestX, bestR)
        print(nonZeroRow.shape)
        print(acc[bestY][bestX][bestR])

        bestRadius = int((np.linspace(minR, maxR, numR))[bestR])

        cv.circle(img, (bestX, bestY), bestRadius, 100)
        cv.imshow("img", img)
        cv.waitKey(30)
    else:
        print("No votes received.")

def callback(data):
    imgData = np.array(data.data, dtype=np.uint8)
    img = np.reshape(imgData, (data.height, data.width))

    img[img > 0] = 255

    hough_circles(img)

def listener():
    rospy.init_node('cylindrical_obstacles', anonymous=True)

    rospy.Subscriber('short_scan', ShortData, callback, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()