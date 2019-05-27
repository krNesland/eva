#!/usr/bin/env python

# Makes a map of the gas concentration.

# Subscribe: /eva/gas_level, /tf

import roslib
import sys
import rospy
import numpy as np
import tf
import math
import cv2 as cv

from std_msgs.msg import Float32

gas_level = 0.0

def callback(data):
    global gas_level
    gas_level = data.data
    print(gas_level)

# Increasing the size of the map and visualizing.
def visualize(gas_map):
    gas_map = gas_map*0.01
    vis_map = cv.resize(gas_map, (1000, 1000))

    cv.imshow("Gas map", vis_map)
    cv.waitKey(3)

# Converting from the map-coordinate system to the image.
def get_map_indices(x, y):
    i = 99 - int(math.floor(10*x))
    j = -int(math.floor(10*y))

    if (i < 0) or (i >= 100):
        i = -1
    if (j < 0) or (j >= 100):
        j = -1
    
    return (i, j)

def map():
    gas_map = np.zeros([100, 100], dtype=np.float32)
    global gas_level
    tf_listener = tf.TransformListener()

    while not rospy.is_shutdown():
        try:
            (trans,rot) = tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            # Position of the robot on the map.
            now_pos = (trans[0], trans[1])
            i, j = get_map_indices(now_pos[0], now_pos[1])

            if (i == -1) or (j == -1):
                print("Estimated position out of map bounds.")
            else:
                # New level at the square is the average of the old level and the new reading.
                gas_map[i][j] = (gas_map[i][j] + gas_level)/2

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        visualize(gas_map)
    
def listener():
    rospy.init_node('gas_map', anonymous=True)

    rospy.Subscriber('/eva/gas_level', Float32, callback)

    map()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()