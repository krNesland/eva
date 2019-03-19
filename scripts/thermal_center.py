#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2 as cv
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import math
import numpy as np

EXPERIMENT = True # Experiment if real life Turtlebot is running.

bridge = CvBridge()

def callback(data):
    try:
        cvImage = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    (rows, cols, channels) = cvImage.shape

    hsv = cv.cvtColor(cvImage, cv.COLOR_BGR2HSV)

    lowerBlue = np.array([110,50,50])
    upperBlue = np.array([130,255,255])

    mask = cv.inRange(hsv, lowerBlue, upperBlue)

    # Vectorized version of finding the centroid.

    xv, yv = np.meshgrid(range(0, cols), range(0, rows))

    xm = np.multiply(xv, mask)
    ym = np.multiply(yv, mask)

    total = np.sum(mask)
    xMoment = np.sum(xm)
    yMoment = np.sum(ym)

    if not (total == 0):
        xArm = xMoment/total
        yArm = yMoment/total
    else:
        xArm = cols/2
        yArm = rows/2

    xArm = int(math.floor(xArm))
    yArm = int(math.floor(yArm))

    cv.drawMarker(cvImage, (xArm, yArm), (0, 0, 255))

    cv.imshow("Image window", cvImage)
    cv.waitKey(3)
    
def listener():
    rospy.init_node('thermal_center', anonymous=True)

    if EXPERIMENT:
        rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, callback, queue_size=1)
    else:
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, callback, queue_size=1) # Simulation.    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()