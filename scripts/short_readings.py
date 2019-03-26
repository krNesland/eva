#!/usr/bin/env python

import roslib
import sys
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from eva_a.msg import *
import numpy as np
import tf
import cv2 as cv
import math
import matplotlib.pyplot as plt

# /scan is a list of 360 ranges (polar coordinates).
# /map is an occupancy grid map (matrix).

nowPose = [0.0, 0.0, 0.0]
mapData = np.zeros((384, 384), dtype=np.int8)
pub = rospy.Publisher('short_scan', ShortData, queue_size=10)

def scan_callback(data):
    rospy.sleep(2) # Let the correct pose be set.

    global nowPose
    global mapData
    global pub
    shortData = np.zeros((384, 384), dtype=np.int8)
    distDelta = 0.3 # How much shorter a reading should be compared to the expected to be considered a possible obstacle.

    maxRange = data.range_max
    minRange = data.range_min

    angleMin = data.angle_min
    angleMax = data.angle_max
    angleInc = data.angle_increment

    thetaB = nowPose[2]
    xB = nowPose[0]
    yB = nowPose[1]

    print("New scan received.")

    for i, angle in enumerate(np.arange(angleMin, angleMax, angleInc, dtype=np.float32)):
        # Angle is the angle of the ray relative to the robot.
        
        scanRange = data.ranges[i]

        # Not exactly sure why 200 and 184. 184 = 384 - 200.
        xI = int(math.floor(20*xB + 200))
        yI = int(math.floor(-20*yB + 184))

        theta = thetaB + angle

        # Line rasterization algorithm 1
        # Positive direction is opposite in image relative to map.

        # Want the angles to be [-pi, pi].
        if theta < math.pi:
            thetaI = -theta
        else:
            thetaI = 2*math.pi - theta

        s = math.tan(thetaI) # dy/dx

        # Starting point for line.
        x = xI
        y = yI

        # Choosing values depending on octant.
        if thetaI > (3/4.0)*math.pi: # 4
            inc = -1
            xAxis = True
        elif thetaI > (2/4.0)*math.pi: # 3
            inc = 1
            xAxis = False
        elif thetaI > (1/4.0)*math.pi: # 2
            inc = 1
            xAxis = False
        elif thetaI > (0/4.0)*math.pi: # 1
            inc = 1
            xAxis = True
        elif thetaI > (-1/4.0)*math.pi: # 8
            inc = 1
            xAxis = True
        elif thetaI > (-2/4.0)*math.pi: # 7
            inc = -1
            xAxis = False
        elif thetaI > (-3/4.0)*math.pi: # 6
            inc = -1
            xAxis = False
        else: # 5
            inc = -1
            xAxis = True

        passedScan = False # If the ray tracing has passed the scan.

        # Tracing until border of map or collision.
        if xAxis:
            while x < 384 and x >= 0 and y < 384 and y >= 0 and mapData[y][x] < 50:
                #mapData[y][x] = 45
                mapRange = math.sqrt((x - xI)*(x - xI) + (y - yI)*(y - yI))/20

                # Storing the position where the ray casting goes further than the scan.
                if (not passedScan) and (mapRange > scanRange):
                    scanPos = (x, y)
                    passedScan = True

                x = x + inc
                y = yI + int(round(s*(x - xI)))
        else:
            while x < 384 and x >= 0 and y < 384 and y >= 0 and mapData[y][x] < 50:
                #mapData[y][x] = 45
                mapRange = math.sqrt((x - xI)*(x - xI) + (y - yI)*(y - yI))/20

                if (not passedScan) and (mapRange > scanRange):
                    scanPos = (x, y)
                    passedScan = True

                y = y + inc
                x = xI + int(round((y - yI)/s))

        mapRange = math.sqrt((x - xI)*(x - xI) + (y - yI)*(y - yI))/20
        
        if mapRange > maxRange:
            mapRange = float('inf')
            
        if mapRange < minRange:
            mapRange = 0
        
        if scanRange < mapRange - distDelta:
            shortData[scanPos[1], scanPos[0]] = 1
    
    if np.any(shortData):
        shortMsg = ShortData()
        shortMsg.header.frame_id = 'opencv'
        shortMsg.height = 384
        shortMsg.width = 384
        shortMsg.data = (shortData.flatten()).tolist()
        pub.publish(shortMsg)

def map_callback(data):
    global mapData
    global nowPose

    img = np.reshape(data.data, (384, 384)).astype(np.int8)
    img = np.flipud(img)
    mapData = img

    # Update pose.
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


def listener():
    rospy.init_node('short_readings', anonymous=True)

    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.Subscriber('/scan', LaserScan, scan_callback, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
