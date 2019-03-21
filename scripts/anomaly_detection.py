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
mapData = np.zeros((384, 384), dtype=np.int8)


def update_pose():
    global nowPose

    tfListener = tf.TransformListener()

    while not rospy.is_shutdown():
        try:
            (trans, rot) = tfListener.lookupTransform(
                '/map', '/base_footprint', rospy.Time(0))
            nowPose[0] = trans[0]
            nowPose[1] = trans[1]
            # Quaternion to theta angle.
            nowPose[2] = math.atan2(
                2*(rot[0]*rot[1] + rot[2]*rot[3]), 1 - 2*(rot[1]*rot[1] + rot[2]*rot[2]))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


def scan_callback(data):
    global nowPose
    global mapData

    rayNum = 45

    r = data.ranges[45]
    thetaL = 45*data.angle_increment
    thetaB = nowPose[2]

    xB = nowPose[0]
    yB = nowPose[1]

    # Not exactly sure why 200 and 184. 184 = 384 - 200.
    xI = int(math.floor(20*xB + 200))
    yI = int(math.floor(-20*yB + 184))

    theta = thetaB + thetaL

    # Line rasterization algorithm 1
    # Positive direction is opposite in image relative to map.
    thetaI = -theta
    s = math.tan(thetaI)

    # Starting point for line.
    x = xI
    y = yI

    if thetaI > (3/4.0)*math.pi: # 4
        inc = -1
        xAxis = True
        print(4)
    elif thetaI > (2/4.0)*math.pi: # 3
        inc = 1
        xAxis = False
        print(3)
    elif thetaI > (1/4.0)*math.pi: # 2
        inc = 1
        xAxis = False
        print(2)
    elif thetaI > (0/4.0)*math.pi: # 1
        inc = 1
        xAxis = True
        print(1)
    elif thetaI > (-1/4.0)*math.pi: # 8
        inc = 1
        xAxis = True
        print(8)
    elif thetaI > (-2/4.0)*math.pi: # 7
        inc = -1
        xAxis = False
        print(7)
    elif thetaI > (-3/4.0)*math.pi: # 6
        inc = -1
        xAxis = False
        print(6)
    else: # 5
        inc = -1
        xAxis = True
        print(5)

    if xAxis:
        while x < 384 and x >= 0 and y < 384 and y >= 0 and mapData[y][x] < 50:
            mapData[y][x] = 45
            x = x + inc
            y = yI + int(round(s*(x - xI)))
    else:
        while x < 384 and x >= 0 and y < 384 and y >= 0 and mapData[y][x] < 50:
            mapData[y][x] = 45
            y = y + inc
            x = xI + int(round((y - yI)/s))

    dist = math.sqrt((x - xI)*(x - xI) + (y - yI)*(y - yI))/20

    print("Map dist: " + str(dist))
    print("Scan dist: " + str(r))

    cv.imshow("Image window", mapData)
    cv.waitKey(3)


def map_callback(data):
    global mapData

    '''
    print('Width:' + str(data.info.width))
    print('Height:' + str(data.info.height))
    print('Resolution:' + str(data.info.resolution))

    print(data.info.origin)

    print(data.data)
    '''

    img = np.reshape(data.data, (384, 384)).astype(np.int8)
    img = np.flipud(img)
    mapData = img


def listener():
    rospy.init_node('anomaly_detection', anonymous=True)

    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    update_pose()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
