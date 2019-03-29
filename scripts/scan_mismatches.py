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

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from eva_a.msg import *

# LaserScan is a list of 360 ranges (polar coordinates).
# OccupancyGrid is an occupancy grid map (matrix).

# Current pose of the robot (x, y, theta).
now_pose = [0.0, 0.0, 0.0]
map_data = np.zeros((384, 384), dtype=np.int8)
obstacle_map = np.zeros((384, 384), dtype = np.float64)
# The occupancy grid. Just a random size for initialization.
pub = rospy.Publisher('/eva/scan_mismatches', ScanMismatches, queue_size=10)

def map_to_img(x_map, y_map):
    x_image = int(round(20*x_map + 200))
    y_image = int(round(-20*y_map + 184))

    return (x_image, y_image)


def map_feature_nearby(x, y, map, region_size):
    left = x - region_size
    if left < 0:
        left = 0

    right = x + region_size
    if right > map.shape[1]:
        right = map.shape[1]

    top = y - region_size
    if top < 0:
        top = 0

    bottom = y + region_size
    if bottom > map.shape[0]:
        bottom = map.shape[0]

    region = map[top:bottom, left:right]

    if np.any(region > 50):
        return True
    else:
        return False

# Loading the map ans setting some parameters
def map_callback(data):
    global map_data
    global now_pose
    global obstacle_map

    map_data=np.reshape(data.data, (384, 384)).astype(np.int8)

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
    map_height = 384
    map_width = 384
    resolution = 0.05

    collision_region = 3

    visualizer = np.zeros((384, 384), np.uint8)

    global now_pose
    global map_data
    global pub
    global obstacle_map

    # Probability of obstacle if hit, but not on map.
    prob_hit_nmap = 0.9
    # Probability of obstacle if hit and on map.
    prob_hit_map = 0.1
    # Probability of obstacle if not hit and not on map.
    prob_nhit_nmap = 0.3
    # Probability of obstacle if not hit and on map.
    prob_nhit_map = 0.01

    max_range=data.range_max
    min_range=data.range_min

    angle_min=data.angle_min
    angle_max=data.angle_max
    angle_inc=data.angle_increment

    robot_theta=now_pose[2]
    robot_x=now_pose[0]
    robot_y=now_pose[1]

    image_x, image_y = map_to_img(robot_x, robot_y)

    print("New scan received.")
    print((robot_x, robot_y))

    # Angle is the angle of the ray relative to the robot. Looping through all.
    for i, angle in enumerate(np.arange(angle_min, angle_max, angle_inc, dtype=np.float32)):

        #cv.imshow("cscsd", np.rot90(visualizer))
        #cv.waitKey(30)
        #rospy.sleep(0.02)

        # Length of the corresponding scan ray.
        scan_range=data.ranges[i]

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

            visualizer[y][x] = 180

            # If the laser scan stops earlier than expected from the map.
            if map_range > (scan_range + collision_region*resolution):
                # If there are cells nearby that are occupied on the map.
                if map_feature_nearby(x, y, map_data, collision_region):
                    update = math.log10(prob_hit_map/(1 - prob_hit_map))
                else:
                    update = math.log10(prob_hit_nmap/(1 - prob_hit_nmap))

                obstacle_map[y][x] = obstacle_map[y][x] + update
                
                break
            else:
                if map_feature_nearby(x, y, map_data, collision_region):
                    update = math.log10(prob_nhit_map/(1 - prob_nhit_map))
                else:
                    update = math.log10(prob_nhit_nmap/(1 - prob_nhit_nmap))

                obstacle_map[y][x] = obstacle_map[y][x] + update

            if x_axis:
                x=x + inc
                y=image_y + int(round(slope*(x - image_x)))
            else:
                y=y + inc
                x=image_x + int(round((y - image_y)/slope))



    obstacle_map[obstacle_map < -2] = -2
    obstacle_map[obstacle_map > 2] = 2

    #print(np.max(obstacle_map))

    visualizer = (obstacle_map - np.min(obstacle_map))/(np.max(obstacle_map) - np.min(obstacle_map))
    visualizer = np.rot90(visualizer)

    cv.imshow("vizz", visualizer)
    cv.waitKey(30)

    '''
    # If there was detected any possible mismatches. Sending a message with these mismatches.
    if np.any(short_data):
        short_msg=ScanMismatches()
        short_msg.header.frame_id='opencv'
        short_msg.height=map_height
        short_msg.width=map_width
        short_msg.data=(short_data.flatten()).tolist()
        pub.publish(short_msg)
    '''


def listener():
    rospy.init_node('scan_mismatches', anonymous = True)

    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.sleep(1)
    rospy.Subscriber('/scan', LaserScan, scan_callback, queue_size = 1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
