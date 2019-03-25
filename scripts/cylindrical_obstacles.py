#!/usr/bin/env python

import roslib
import rospy
from eva_a.msg import *
import numpy as np
import cv2 as cv

def callback(data):
    imgData = np.array(data.data, dtype=np.int8)
    img = np.reshape(imgData, (data.height, data.width))

    img[img > 0] = 100

    cv.imshow("Image window", img)
    cv.waitKey()

def listener():
    rospy.init_node('cylindrical_obstacles', anonymous=True)

    rospy.Subscriber('short_scan', ShortData, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()