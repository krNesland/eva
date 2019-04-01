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
map_data = np.zeros((500, 500), dtype=np.uint8)
obstacle_map = np.zeros((200, 200), dtype = np.float64)
visualizer = np.zeros((200, 200), dtype=np.float64)
# The occupancy grid. Just a random size for initialization.
pub = rospy.Publisher('/eva/scan_mismatches', ScanMismatches, queue_size=10)

#Heisann

# Loading the map ans setting some parameters
def map_callback(data):
    global map_data
    global now_pose
    global obstacle_map
    global visualizer

    try:
        map_width=data.info.width
        map_height=data.info.height
        resolution=data.info.resolution

        rospy.set_param('/eva/mapWidth', map_width)
        rospy.set_param('/eva/mapHeight', map_height)
        rospy.set_param('/eva/mapPixelsPerMeter', 1.0/resolution)

        map_height=rospy.get_param('/eva/mapHeight')
        map_width=rospy.get_param('/eva/mapWidth')
    except:
        print("Unable to load map info.")
        return

    img=np.reshape(data.data, (map_height, map_width)).astype(np.int8)

    # The map is created with a 200 px margin down and to the left.
    y_crop = map_height - 200
    x_crop = 200

    img=img[:(map_height - y_crop), x_crop:]

    map_data=img
    obstacle_map = np.zeros(img.shape, dtype = np.float64)
    visualizer = np.zeros(img.shape, dtype=np.float64)

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
    # Wait for the correct pose to be set.
    rospy.sleep(2)

    try:
        map_height = 200
        map_width = int(rospy.get_param('/eva/mapWidth') - 200)
        map_pixels_per_meter = int(rospy.get_param('/eva/mapPixelsPerMeter'))
    except:
        print("Unable to load map parameters.")
        return

    try:
        # How much shorter a reading should be compared to the expected to be considered a possible obstacle.
        scan_ray_distance_threshold = rospy.get_param(
            '/eva/scanRayDistanceThreshold')
    except:
        scan_ray_distance_threshold = 0.3
        print("Scan ray distance threshold defaulting to 0.3 m.")

    global now_pose
    global map_data
    global pub
    global obstacle_map
    global visualizer

    # Probability of obstacle if hit, but not on map.
    prob_hit_nmap = 0.95
    # Probability of obstacle if hit and on map.
    prob_hit_map = 0.01
    # Probability of obstacle if not hit and not on map.
    prob_nhit_nmap = 0.45
    # Probability of obstacle if not hit and on map.
    prob_nhit_map = 0.01

    prob_hit = 0.8
    prob_nhit = 0.4

    max_range=data.range_max
    min_range=data.range_min

    angle_min=data.angle_min
    angle_max=data.angle_max
    angle_inc=data.angle_increment

    robot_theta=now_pose[2]
    robot_x=now_pose[0]
    robot_y=now_pose[1]

    print("New scan received.")

    # Angle is the angle of the ray relative to the robot. Looping through all.
    for i, angle in enumerate(np.arange(angle_min, angle_max, angle_inc, dtype=np.float32)):

        # Length of the corresponding scan ray.
        scan_range=data.ranges[i]

        image_x=int(math.floor(map_pixels_per_meter*robot_x))
        image_y=int(math.floor(map_pixels_per_meter*robot_y + map_height))

        image_theta=robot_theta + angle

        # Line rasterization algorithm 1
        # Positive direction is opposite in image relative to map.

        # Want the angles to be [-pi, pi].
        if image_theta > math.pi:
            image_theta= image_theta - 2*math.pi 

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
        if x_axis:
            while x < map_width and x >= 0 and y < map_height and y >= 0:
                map_range=math.sqrt(
                    (x - image_x)*(x - image_x) + (y - image_y)*(y - image_y))/map_pixels_per_meter

                '''
                # Do not want to continue following a ray long after it should have hit something.
                if map_range > scan_range + 0.1:
                    break

                # If map is occupied.
                if map_data[y][x] > 50:
                    # If scan hits at approximately this position.
                    if abs(map_range - scan_range) < 0.1:
                        update = math.log10(prob_hit_map/(1 - prob_hit_map))
                    else:
                        update = math.log10(prob_nhit_map/(1 - prob_nhit_map))
                else:
                    # If scan hits at approximately this position.
                    if abs(map_range - scan_range) < 0.1:
                        update = math.log10(prob_hit_nmap/(1 - prob_hit_nmap))
                    else:
                        update = math.log10(prob_nhit_nmap/(1 - prob_nhit_nmap))

                obstacle_map[y][x] = obstacle_map[y][x] + update
                '''

                if map_range > scan_range:
                    visualizer[y][x] = visualizer[y][x] + math.log10(prob_hit/(1 - prob_hit))
                    break
                else:
                    visualizer[y][x] = visualizer[y][x] + math.log10(prob_nhit/(1 - prob_nhit))

                x=x + inc
                y=image_y + int(round(slope*(x - image_x)))
        else:
            while x < map_width and x >= 0 and y < map_height and y >= 0:
                map_range=math.sqrt((x - image_x)*(x - image_x) + (y - image_y)*(y - image_y))/map_pixels_per_meter

                '''
                # Do not want to continue following a ray long after it should have hit something.
                if map_range > scan_range + 0.1:
                    break

                # If map is occupied.
                if map_data[y][x] > 50:
                    # If scan hits at approximately this position.
                    if abs(map_range - scan_range) < 0.2:
                        update = math.log10(prob_hit_map/(1 - prob_hit_map))
                    else:
                        update = math.log10(prob_nhit_map/(1 - prob_nhit_map))
                else:
                    # If scan hits at approximately this position.
                    if abs(map_range - scan_range) < 0.2:
                        update = math.log10(prob_hit_nmap/(1 - prob_hit_nmap))
                    else:
                        update = math.log10(prob_nhit_nmap/(1 - prob_nhit_nmap))

                obstacle_map[y][x] = obstacle_map[y][x] + update
                '''

                if map_range > scan_range:
                    visualizer[y][x] = visualizer[y][x] + math.log10(prob_hit/(1 - prob_hit))
                    break
                else:
                    visualizer[y][x] = visualizer[y][x] + math.log10(prob_nhit/(1 - prob_nhit))

                y=y + inc
                x=image_x + int(round((y - image_y)/slope))

    print(np.min(obstacle_map))
    
    #visualizer = np.rot90(np.flipud(visualizer))
    obstacle_map[obstacle_map < -10] = -10

    visualizer[visualizer < -10] = -10

    #cv.imshow("myWin", (obstacle_map - np.min(obstacle_map))/(np.max(obstacle_map) - np.min(obstacle_map)))
    cv.imshow("viz", (visualizer - np.min(visualizer))/(np.max(visualizer) - np.min(visualizer)))
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
    rospy.Subscriber('/scan', LaserScan, scan_callback, queue_size = 1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()