#!/usr/bin/env python

import roslib
import sys
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import cv2 as cv
import numpy as np
import tf
import math

# /scan is a list of 360 ranges (polar coordinates).
#

# Will need the scan and the expected range along every ray from the map as input.

nowPose = [0.0, 0.0, 0.0]

def update_pose():
    global nowPose

    tfListener = tf.TransformListener()

    while not rospy.is_shutdown():
        try:
            (trans,rot) = tfListener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            nowPose[0] = trans[0]
            nowPose[1] = trans[1]
            nowPose[2] = math.atan2(2*(rot[0]*rot[1] + rot[2]*rot[3]), 1 - 2*(rot[1]*rot[1] + rot[2]*rot[2])) # Quaternion to theta angle.
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


def scan_callback(data):
    global nowPose

    print(nowPose)

def map_callback(data):
    '''
    print('Width:' + str(data.info.width))
    print('Height:' + str(data.info.height))
    print('Resolution:' + str(data.info.resolution))

    print(data.info.origin)

    print(data.data)
    '''

    img = np.reshape(data.data, (384, 384)).astype(np.int8)
    img = np.flipud(img)
    print(data.info.origin)
    cv.imshow("Image window", img)
    cv.waitKey()

def listener():
    rospy.init_node('anomaly_detection', anonymous=True)

    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    update_pose()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()