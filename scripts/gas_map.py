#!/usr/bin/env python

import roslib
import sys
import rospy
from std_msgs.msg import String
import numpy as np
import tf
import math
import cv2 as cv

gasLevel = 0.0

def callback(data):
    global gasLevel
    gasLevel = float(data.data)
    print(gasLevel)

def visualize(gasMap):
    gasMap = gasMap*0.01
    visMap = cv.resize(gasMap, (1000, 1000))

    cv.imshow("Gas map", visMap)
    cv.waitKey(3)

def get_map_indices(x, y):
    i = 99 - int(math.floor(10*x))
    j = -int(math.floor(10*y))

    if (i < 0) or (i >= 100):
        i = -1
    if (j < 0) or (j >= 100):
        j = -1
    
    return (i, j)

def map():
    gasMap = np.zeros([100, 100], dtype=np.float32)
    global gasLevel
    tfListener = tf.TransformListener()

    while not rospy.is_shutdown():
        try:
            (trans,rot) = tfListener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            nowPos = (trans[0], trans[1])
            i, j = get_map_indices(nowPos[0], nowPos[1])

            if (i == -1) or (j == -1):
                print("Estimated position out of map bounds.")
            else:
                gasMap[i][j] = (gasMap[i][j] + gasLevel)/2

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        visualize(gasMap)
    
def listener():
    rospy.init_node('gas_map', anonymous=True)

    rospy.Subscriber('/eva/gas_level', String, callback)

    map()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()