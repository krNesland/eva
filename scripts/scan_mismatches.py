#!/usr/bin/env python

# Performs ray tracing on the map and compares the distance with the distance of the scan. If the scan is considerably shorter, this is marked as a potential obstacle.

# Subscribe: /scan, /map
# Publish: /eva/short_scan

import roslib
import rospy
import numpy as np
import tf
import math

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from eva_a.msg import *

# LaserScan is a list of 360 ranges (polar coordinates).
# OccupancyGrid is an occupancy grid map (matrix).

# Current pose of the robot (x, y, theta).
now_pose = [0.0, 0.0, 0.0]
# The occupancy grid. Just a random size for initialization.
map_data = np.zeros((500, 500), dtype=np.int8)
pub = rospy.Publisher('/eva/scan_mismatches', ScanMismatches, queue_size=10)

# Comparing scan to map.
def scan_callback(data):
    # Wait for the correct pose to be set.
    rospy.sleep(2)

    try:
        map_height = int(rospy.get_param('/eva/mapHeight'))
        map_width = int(rospy.get_param('/eva/mapHeight'))
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
    short_data = np.zeros((map_height, map_width), dtype = np.int8)

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

        # Not exactly sure why 200 and 184. 184 = 384 - 200.
        image_x=int(math.floor(map_pixels_per_meter*robot_x + 200))
        image_y=int(math.floor(-map_pixels_per_meter*robot_y + map_height - 200))

        image_theta=robot_theta + angle

        # Line rasterization algorithm 1
        # Positive direction is opposite in image relative to map.

        # Want the angles to be [-pi, pi].
        if image_theta < math.pi:
            image_theta=-image_theta
        else:
            image_theta=2*math.pi - image_theta

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

        # If the ray tracing has passed the scan.
        passed_scan=False

        # Tracing until border of map or collision.
        if x_axis:
            while x < map_width and x >= 0 and y < map_height and y >= 0 and map_data[y][x] < 50:
                map_range=math.sqrt(
                    (x - image_x)*(x - image_x) + (y - image_y)*(y - image_y))/map_pixels_per_meter

                # Storing the position where the ray casting goes further than the scan.
                if (not passed_scan) and (map_range > scan_range):
                    scan_pos=(x, y)
                    passed_scan=True

                x=x + inc
                y=image_y + int(round(slope*(x - image_x)))
        else:
            while x < map_width and x >= 0 and y < map_height and y >= 0 and map_data[y][x] < 50:
                map_range=math.sqrt((x - image_x)*(x - image_x) + (y - image_y)*(y - image_y))/map_pixels_per_meter

                if (not passed_scan) and (map_range > scan_range):
                    scan_pos=(x, y)
                    passed_scan=True

                y=y + inc
                x=image_x + int(round((y - image_y)/slope))

        map_range=math.sqrt((x - image_x)*(x - image_x) + (y - image_y)*(y - image_y))/map_pixels_per_meter

        if map_range > max_range:
            map_range=float('inf')

        if map_range < min_range:
            map_range=0

        # If a possible mismatch.
        if scan_range < map_range - scan_ray_distance_threshold:
            short_data[scan_pos[1], scan_pos[0]]=1

    # If there was detected any possible mismatches. Sending a message with these mismatches.
    if np.any(short_data):
        short_msg=ShortData()
        short_msg.header.frame_id='opencv'
        short_msg.height=map_height
        short_msg.width=map_width
        short_msg.data=(short_data.flatten()).tolist()
        pub.publish(short_msg)


# Loading the map ans setting some parameters
def map_callback(data):
    global map_data
    global now_pose

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
    img=np.flipud(img)
    map_data=img

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


def listener():
    rospy.init_node('scan_mismatches', anonymous = True)

    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.Subscriber('/scan', LaserScan, scan_callback, queue_size = 1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
