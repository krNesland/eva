#!/usr/bin/env python

# Performs ray tracing on the map and compares the distance with the distance of the real scan. If the scan is considerably shorter, this is marked as a potential obstacle.

# Subscribe: /scan, /map, /tf
# Publish: /eva/scan_mismatches

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
map_old = np.zeros((384, 384), dtype=np.uint8)
map_new = np.zeros((384, 384), dtype = np.float32)
# The occupancy grid. Just a random size for initialization.
pub = rospy.Publisher('/eva/scan_mismatches', ScanMismatches, queue_size=10)

# Transformation from object frame to camera frame.
def map_to_img(x, y):
    u = int(round(20*x + 200)) - 1
    v = int(round(-20*y + 184)) - 1

    return (u, v)

# Loading the map and getting some parameters
def map_callback(data):
    global map_old
    global now_pose
    global map_new

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
        map_dilate_radius = 6

    # Making it equal to how the .pgm file looks.
    raw_map_old = np.flipud(np.reshape(data.data, (map_height, map_width)).astype(np.int8))
    raw_map_old[raw_map_old < 0] = 0
    raw_map_old = raw_map_old.astype(np.uint8)

    ret, map_old = cv.threshold(raw_map_old, 50, 255, cv.THRESH_BINARY)
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2*map_dilate_radius, 2*map_dilate_radius))
    map_old = cv.morphologyEx(map_old, cv.MORPH_DILATE, kernel)

    # Reading and storing the pose. Keeps on going until the node is stopped.
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

# Performing new mapping.
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
    global map_old
    global pub
    global map_new

    # Loading parameters for the update of the occupancy grid map.
    try:
         # Probability of occupied if hit.
        prob_hit = rospy.get_param('/eva/probOccIfHit')
        # Probability of occupied if not hit.
        prob_nhit = rospy.get_param('/eva/probOccIfNotHit')
    except:
        print("Unable to load occupancy parameters.")
        prob_hit = 0.75
        prob_nhit = 0.45

    # Parameters for the laser scan.
    max_range=data.range_max
    min_range=data.range_min
    angle_min=data.angle_min
    angle_max=data.angle_max
    angle_inc=data.angle_increment

    # Getting the current pose of the robot in the object frame.
    robot_theta=now_pose[2]
    robot_x=now_pose[0]
    robot_y=now_pose[1]

    # Starting point of line in image (not u0 and v0 of the camera model, but 0 as in initial).
    u0, v0 = map_to_img(robot_x, robot_y)

    # Angle is the angle of the ray in the robot frame. Looping through all.
    for i, angle in enumerate(np.arange(angle_min, angle_max, angle_inc, dtype=np.float32)):

        # Length of the corresponding, real, scan ray.
        scan_range=data.ranges[i]

        # Do not want to analyze the directions where the readings have errors.
        if scan_range < (min_range + 0.01):
            continue

        # Finding the angle in the camera frame.
        image_theta= -(robot_theta + angle)

        # Want the angles to be [-pi, pi].
        if image_theta > math.pi:
            image_theta = image_theta - 2*math.pi
        elif image_theta < -math.pi:
            image_theta = 2*math.pi + image_theta

        # dy/dx (slope of the line).
        slope=math.tan(image_theta)

        # Starting point for line in the camera frame.
        u=u0
        v=v0

        # Choosing values depending on octant.
        if image_theta > (3/4.0)*math.pi:  # 4
            inc=-1
            u_axis=True
        elif image_theta > (2/4.0)*math.pi:  # 3
            inc=1
            u_axis=False
        elif image_theta > (1/4.0)*math.pi:  # 2
            inc=1
            u_axis=False
        elif image_theta > (0/4.0)*math.pi:  # 1
            inc=1
            u_axis=True
        elif image_theta > (-1/4.0)*math.pi:  # 8
            inc=1
            u_axis=True
        elif image_theta > (-2/4.0)*math.pi:  # 7
            inc=-1
            u_axis=False
        elif image_theta > (-3/4.0)*math.pi:  # 6
            inc=-1
            u_axis=False
        else:  # 5
            inc=-1
            u_axis=True

        # Tracing until border of map.
        while u < map_width and u >= 0 and v < map_height and v >= 0:
            # Length of the traced line (so far).
            map_range=math.sqrt(
                (u - u0)*(u - u0) + (v - v0)*(v - v0))*resolution

            # No point in tracing longer than the max range of the LiDAR.
            if map_range > max_range:
                break

            # If the traced ray has become longer than the reported LiDAR ray (has just collided).
            if map_range > scan_range:
                update = math.log10(prob_hit/(1 - prob_hit))
                map_new[v][u] = map_new[v][u] + update

                break
            # Else, the cell appears to be unoccupied.
            else:
                update = math.log10(prob_nhit/(1 - prob_nhit))
                map_new[v][u] = map_new[v][u] + update

            # Moving on to the next cell.
            if u_axis:
                u=u + inc
                v=v0 + int(round(slope*(u - u0)))
            else:
                v=v + inc
                u=u0 + int(round((v - v0)/slope))

    # Setting upper and lower threshold for the log odds.
    map_new[map_new < -2] = -2
    map_new[map_new > 2] = 2

    # Normalizing (zero to one).
    map_new_norm = (map_new + 2)/4

    visualizer = np.copy(map_new_norm)
    
    # Removing cells that were already occupied in the original mapping.
    map_new_norm[map_old > 127] = 0.0

    # Converting to integer.
    map_new_norm = np.array(255.0*map_new_norm, dtype=np.uint8)
    
    visualizer2 = np.copy(map_new_norm)

    cv.imshow("Dilated map", map_old)
    cv.imshow("Raw mapping", visualizer)
    cv.imshow("Subtracted mapping", visualizer2)
    cv.waitKey(30)

    # If there was detected any possible mismatches. Sending a message with these mismatches.
    if np.any(map_new_norm):
        mismatch_msg=ScanMismatches()
        mismatch_msg.header.frame_id='opencv'
        mismatch_msg.height=map_height
        mismatch_msg.width=map_width
        mismatch_msg.data=(map_new_norm.flatten()).tolist()
        pub.publish(mismatch_msg)

def listener():
    rospy.init_node('scan_mismatches', anonymous = True)

    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.sleep(1)
    rospy.Subscriber('/scan', LaserScan, scan_callback, queue_size = 1)

    rospy.spin()

if __name__ == '__main__':
    listener()