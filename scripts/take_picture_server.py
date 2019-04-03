#!/usr/bin/env python

# Takes a picture of an obstacle given its position.

# Subscribe: 

import roslib
import rospy
import numpy as np
import cv2 as cv
import math
import tf

from nav_msgs.msg import OccupancyGrid
from eva_a.srv import *


map_data = np.zeros((384, 384), dtype=np.int8)
robot_pose = [0.0, 0.0, 0.0]

def map_callback(data):
    global map_data
    global robot_pose

    map_data = np.flipud(np.reshape(data.data, (data.info.height, data.info.width)).astype(np.int8))

    # Update pose.
    tf_listener=tf.TransformListener()

    while not rospy.is_shutdown():
        try:
            (trans, rot)=tf_listener.lookupTransform(
                '/map', '/base_footprint', rospy.Time(0))
            robot_pose[0]=trans[0]
            robot_pose[1]=trans[1]
            # Quaternion to theta angle.
            robot_pose[2]=math.atan2(
                2*(rot[0]*rot[1] + rot[2]*rot[3]), 1 - 2*(rot[1]*rot[1] + rot[2]*rot[2]))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


# Converting from map coordinates to coordinates of map_data.
def map_to_img(x_map, y_map):
    x_image = int(round(20*x_map + 200))
    y_image = int(round(-20*y_map + 184))

    return (x_image, y_image)


def free_space(x_map, y_map, region_size):
    global map_data
    x, y = map_to_img(x_map, y_map)

    left = x - region_size
    if left < 0:
        return False

    right = x + region_size
    if right > map_data.shape[1]:
        return False

    top = y - region_size
    if top < 0:
        return False

    bottom = y + region_size
    if bottom > map_data.shape[0]:
        return False

    region = map_data[top:bottom, left:right]

    if np.any(region > 50):
        return False
    else:
        return True



def find_capturing_pose(x_obstacle, y_obstacle):
    global map_data

    step = (2*math.pi)/10
    possible_poses = []


    for i in range(10):
        angle = i*step

        x = x_obstacle + math.cos(angle)
        y = y_obstacle + math.sin(angle)
        theta = angle - math.pi

        if free_space(x, y, 6):
            possible_poses.append((x, y, theta))

    visualizer = np.zeros((384, 384), dtype=np.uint8)
    visualizer[map_data > 50] = 50
    for tup in possible_poses:
        vizX, vizY = map_to_img(tup[0], tup[1])
        visualizer[vizY][vizX] = 200

    cv.imshow("vizz", visualizer)
    cv.waitKey()

    # Velg den nærmeste. Instruer om å kjøre dit og ta bilde når nærme nok.


def handle_take_picture(req):
    global map_data

    x_obstacle = req.obstaclePosX
    y_obstacle = req.obstaclePosY

    find_capturing_pose(x_obstacle, y_obstacle)

    return TakePictureResponse(1)


def take_picture_server():
    rospy.init_node('take_picture', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, map_callback, queue_size=1)

    s = rospy.Service('/eva/take_picture',
                      TakePicture, handle_take_picture)
    rospy.spin()


if __name__ == "__main__":
    take_picture_server()
