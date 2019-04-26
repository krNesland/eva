#!/usr/bin/env python

# Performs ray tracing on the map and compares the distance with the distance of the scan. If the scan is considerably shorter, this is marked as a potential obstacle.

# Subscribe: /scan, /map
# Publish: /eva/short_scan

import roslib
import rospy
import numpy as np
import tf
import math
import cv2 as cv
import matplotlib.pyplot as plt

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from eva_a.msg import *

# LaserScan is a list of 360 ranges (polar coordinates).
# OccupancyGrid is an occupancy grid map (matrix).

# Current pose of the robot (x, y, theta).
now_pose = [0.0, 0.0, 0.0]
map_data = np.zeros((384, 384), dtype=np.uint8)
obstacle_map = np.zeros((384, 384), dtype = np.float32)
# The occupancy grid. Just a random size for initialization.
pub = rospy.Publisher('/eva/scan_mismatches', ScanMismatches, queue_size=10)

def map_to_img(x_map, y_map):
    x_image = int(round(20*x_map + 200)) - 1
    y_image = int(round(-20*y_map + 184)) - 1

    return (x_image, y_image)

# Loading the map ans setting some parameters
def map_callback(data):
    global map_data
    global now_pose
    global obstacle_map

    try:
        map_width = rospy.get_param('/eva/mapWidth')
        map_height = rospy.get_param('/eva/mapHeight')
        resolution = rospy.get_param('/eva/resolution')
        map_dilate_radius = rospy.get_param('/eva/mapDilateRadius')
    except:
        print("Unable to load map parameters.")
        map_width = 384
        map_height = 384
        resolution = 0.05
        map_dilate_radius = 5

    # Making it equal to how the .pgm file looks.
    raw_map_data = np.flipud(np.reshape(data.data, (map_height, map_width)).astype(np.int8))
    raw_map_data[raw_map_data < 0] = 0
    raw_map_data = raw_map_data.astype(np.uint8)

    ret, map_data = cv.threshold(raw_map_data, 50, 255, cv.THRESH_BINARY)
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (map_dilate_radius, map_dilate_radius))
    map_data = cv.morphologyEx(map_data, cv.MORPH_DILATE, kernel)

    # Update pose.
    tf_listener=tf.TransformListener()

    while not rospy.is_shutdown():
        try:
            (trans, rot)=tf_listener.lookupTransform(
                '/map', '/base_footprint', rospy.Time(0))
            now_pose[0]=trans[0]
            now_pose[1]=trans[1]
            # Quaternion to theta angle.
            now_pose[2]=math.atan2(
                2*(rot[0]*rot[1] + rot[2]*rot[3]), 1 - 2*(rot[1]*rot[1] + rot[2]*rot[2]))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


# Comparing scan to map.
def scan_callback(data):
    try:
        map_width = rospy.get_param('/eva/mapWidth')
        map_height = rospy.get_param('/eva/mapHeight')
        resolution = rospy.get_param('/eva/resolution')
    except:
        map_width = 384
        map_height = 384
        resolution = 0.05

    global now_pose
    global map_data
    global pub
    global obstacle_map

    try:
         # Probability of occupied if hit.
        prob_hit = rospy.get_param('/eva/probOccIfHit')
        # Probability of occupied if not hit.
        prob_nhit = rospy.get_param('/eva/probOccIfNotHit')
    except:
        print("Unable to load occupancy parameters.")
         # Probability of occupied if hit.
        prob_hit = 0.65
        # Probability of occupied if not hit.
        prob_nhit = 0.45

    max_range=data.range_max
    min_range=data.range_min

    angle_min=data.angle_min
    angle_max=data.angle_max
    angle_inc=data.angle_increment

    robot_theta=now_pose[2]
    robot_x=now_pose[0]
    robot_y=now_pose[1]

    image_x, image_y = map_to_img(robot_x, robot_y)

    # Angle is the angle of the ray relative to the robot. Looping through all.
    for i, angle in enumerate(np.arange(angle_min, angle_max, angle_inc, dtype=np.float32)):

        # Length of the corresponding scan ray.
        scan_range=data.ranges[i]

        # Do not want to analyze the directions where the readings have errors.
        if scan_range < (min_range + 0.01):
            continue

        image_theta= -(robot_theta + angle)

        # Line rasterization algorithm 1

        # Want the angles to be [-pi, pi].
        if image_theta > math.pi:
            image_theta = image_theta - 2*math.pi
        elif image_theta < -math.pi:
            image_theta = 2*math.pi + image_theta

        # dy/dx (slope of the line).
        slope=math.tan(image_theta)

        # Starting point for line.
        x=image_x
        y=image_y

        # Choosing values depending on octant.
        if image_theta > (3/4.0)*math.pi:  # 4
            inc=-1
            x_axis=True
        elif image_theta > (2/4.0)*math.pi:  # 3
            inc=1
            x_axis=False
        elif image_theta > (1/4.0)*math.pi:  # 2
            inc=1
            x_axis=False
        elif image_theta > (0/4.0)*math.pi:  # 1
            inc=1
            x_axis=True
        elif image_theta > (-1/4.0)*math.pi:  # 8
            inc=1
            x_axis=True
        elif image_theta > (-2/4.0)*math.pi:  # 7
            inc=-1
            x_axis=False
        elif image_theta > (-3/4.0)*math.pi:  # 6
            inc=-1
            x_axis=False
        else:  # 5
            inc=-1
            x_axis=True

        # Tracing until border of map.
        while x < map_width and x >= 0 and y < map_height and y >= 0:
            map_range=math.sqrt(
                (x - image_x)*(x - image_x) + (y - image_y)*(y - image_y))*resolution

            if map_range > max_range:
                break

            # If the laser scan stops earlier than expected from the map.
            if map_range > scan_range:
                update = math.log10(prob_hit/(1 - prob_hit))
                obstacle_map[y][x] = obstacle_map[y][x] + update

                break
            else:
                update = math.log10(prob_nhit/(1 - prob_nhit))
                obstacle_map[y][x] = obstacle_map[y][x] + update

            if x_axis:
                x=x + inc
                y=image_y + int(round(slope*(x - image_x)))
            else:
                y=y + inc
                x=image_x + int(round((y - image_y)/slope))
        

    obstacle_map[obstacle_map < -2] = -2
    obstacle_map[obstacle_map > 2] = 2

    visualizer = (obstacle_map + 2)/4
    visualizer[map_data > 127] = 0.0 # Removing where there is map

    visualizer2 = np.array(255.0*visualizer, dtype=np.uint8)

    cv.imshow("visualizer", visualizer)
    cv.waitKey(30)

    # If there was detected any possible mismatches. Sending a message with these mismatches.
    if np.any(visualizer2):
        mismatch_msg=ScanMismatches()
        mismatch_msg.header.frame_id='opencv'
        mismatch_msg.height=map_height
        mismatch_msg.width=map_width
        mismatch_msg.data=(visualizer2.flatten()).tolist()
        pub.publish(mismatch_msg)


def listener():
    rospy.init_node('scan_mismatches', anonymous = True)

    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.sleep(1)
    rospy.Subscriber('/scan', LaserScan, scan_callback, queue_size = 1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
