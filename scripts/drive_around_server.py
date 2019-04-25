#!/usr/bin/env python

# Takes a picture of an obstacle given its position.

# Subscribe: 

import roslib
import rospy
import numpy as np
import cv2 as cv
import math
import tf

from eva_a.msg import *
from eva_a.srv import *
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Twist


obstacle_map = np.zeros((384, 384), dtype=np.uint8)
robot_pose = [0.0, 0.0, 0.0]

def map_callback(data):
    global obstacle_map
    global robot_pose

    obstacle_data = np.array(data.data, dtype=np.uint8)
    obstacle_map = np.reshape(obstacle_data, (data.height, data.width))
    ret, obstacle_map = cv.threshold(obstacle_map, 150, 255, cv.THRESH_BINARY)

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


# Converting from map coordinates to coordinates of obstacle_map.
def map_to_img(x_map, y_map):
    x_image = int(round(20*x_map + 199))
    y_image = int(round(-20*y_map + 183))

    return (x_image, y_image)


# Extracts the region of the obstacle map that has just been circled around.
def extract_region(x_obstacle, y_obstacle, circling_radius):
    global obstacle_map

    width = int(round(20*2*circling_radius))
    height = width
    left, bottom = map_to_img(x_obstacle - circling_radius, y_obstacle - circling_radius)

    # Look
    region = obstacle_map[(bottom - height):bottom, left:(left + width)]
    print((left, bottom))
    print(region.shape)

    cv.imshow('obstacle', region)
    cv.waitKey()


def free_space(x_map, y_map, region_size):
    global obstacle_map
    x, y = map_to_img(x_map, y_map)

    left = x - region_size
    if left < 0:
        return False

    right = x + region_size
    if right > obstacle_map.shape[1]:
        return False

    top = y - region_size
    if top < 0:
        return False

    bottom = y + region_size
    if bottom > obstacle_map.shape[0]:
        return False

    region = obstacle_map[top:bottom, left:right]

    if np.any(region > 50):
        return False
    else:
        return True


# Should also be able to define the wanted radius.
def drive_around(radius):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.sleep(0.5)

    print("Driving around.")

    # Driving around.
    vel_msg_around = Twist()
    vel_msg_around.angular.z = 0.125/radius
    vel_msg_around.linear.x = 0.125
    pub.publish(vel_msg_around)
    rospy.sleep(16*3.14*radius)

    vel_msg_stop = Twist()
    pub.publish(vel_msg_stop)
    print("Finished driving around.")

# Almost identical to find_capturing_pose().
def find_starting_pose(x_obstacle, y_obstacle, circling_radius):
    global obstacle_map
    global robot_pose

    step = (2*math.pi)/10
    possible_poses = []

    for i in range(10):
        angle = i*step

        x = x_obstacle + circling_radius*math.cos(angle)
        y = y_obstacle + circling_radius*math.sin(angle)
        theta = angle - math.pi - (math.pi/2)

        # If a 12x12 px area unoccupied around.
        if free_space(x, y, 6):
            possible_poses.append((x, y, theta))


    '''

    visualizer = np.zeros((384, 384), dtype=np.uint8)
    visualizer[obstacle_map > 50] = 50

    obstacleX, obstacleY = map_to_img(x_obstacle, y_obstacle)
    visualizer[obstacleY][obstacleX] = 250

    for tup in possible_poses:
        vizX, vizY = map_to_img(tup[0], tup[1])
        visualizer[vizY][vizX] = 150

    cv.imshow("vizz", visualizer)
    cv.waitKey()

    '''



    # Just some high value.
    shortest_dist = 1000

    # Finding the closest capturing position from where the robot is currently at.
    for pose in possible_poses:
        dist = math.sqrt((robot_pose[0] - pose[0])**2 + (robot_pose[1] - pose[1])**2)

        if dist < shortest_dist:
            selected_pose = pose
            shortest_dist = dist

    if len(possible_poses) > 0:
        return selected_pose
    else:
        return []


def navigate_to_pose(pose):
    global robot_pose

    pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
    msg = MoveBaseActionGoal()
    msg.goal.target_pose.header.frame_id = "map"
    msg.goal.target_pose.pose.position.x = pose[0]
    msg.goal.target_pose.pose.position.y = pose[1]
    # Orientation as a quaternion.
    msg.goal.target_pose.pose.orientation.z = math.sin(pose[2]/2)
    msg.goal.target_pose.pose.orientation.w = math.cos(pose[2]/2)

    rospy.sleep(0.5)
    pub.publish(msg)

    # Close enough to the wanted pose yet?
    finished = False

    while (not rospy.is_shutdown()) and (not finished):
        dist = dist = math.sqrt((robot_pose[0] - pose[0])**2 + (robot_pose[1] - pose[1])**2)

        if dist < 0.1:
            finished = True

        rospy.sleep(1.0)
    
    rospy.sleep(1.0)

def handle_drive_around(req):
    global obstacle_map

    circling_radius = 0.7

    x_obstacle = req.obstaclePosX
    y_obstacle = req.obstaclePosY

    pose = find_starting_pose(x_obstacle, y_obstacle, circling_radius)
    print(pose)

    if len(pose) > 0:
        navigate_to_pose(pose)

        rospy.sleep(1)

        drive_around(circling_radius)
        extract_region(x_obstacle, y_obstacle, circling_radius)

        return TakePictureResponse(1)
    else:
        print("Could not find a suitable pose to start from.")
        return TakePictureResponse(0)


def take_picture_server():
    rospy.init_node('take_picture', anonymous=True)
    rospy.Subscriber('/eva/scan_mismatches',
                     ScanMismatches, map_callback, queue_size=1)

    s = rospy.Service('/eva/take_picture',
                      TakePicture, handle_drive_around)
    rospy.spin()


if __name__ == "__main__":
    take_picture_server()
