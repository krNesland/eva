#!/usr/bin/env python

# Takes a picture of an obstacle given its position.

# Subscribe: 

import roslib
import rospy
roslib.load_manifest('eva_a')
import actionlib
import numpy as np
import cv2 as cv
import math
import tf

from eva_a.msg import *
from eva_a.srv import *
from geometry_msgs.msg import Twist
from move_base_msgs.msg import *

obstacle_map = np.zeros((384, 384), dtype=np.uint8)

def map_callback(data):
    global obstacle_map

    obstacle_data = np.array(data.data, dtype=np.uint8)
    obstacle_map = np.reshape(obstacle_data, (data.height, data.width))
    ret, obstacle_map = cv.threshold(obstacle_map, 150, 255, cv.THRESH_BINARY)

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

    region = obstacle_map[(bottom - height):bottom, left:(left + width)]
    print((left, bottom))
    print(region.shape)

    return region


def find_boundary(region):
    # Close to connect.
    se_close = cv.getStructuringElement(cv.MORPH_ELLIPSE, (7, 7))
    region_closed = cv.morphologyEx(region, cv.MORPH_CLOSE, se_close)

    # Find contours.
    im2, contours, hierarchy = cv.findContours(region_closed, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    # Find the largest structure (in case there are several structures inside the region).
    cnt = contours[0]
    max_area = cv.contourArea(cnt)

    for cont in contours:
        if cv.contourArea(cont) > max_area:
            cnt = cont
            max_area = cv.contourArea(cont)

    approx = cv.approxPolyDP(cnt, 1.0, True)

    print(cnt)
    print(approx)

    return approx

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

    step = (2*math.pi)/10
    possible_poses = []

    for i in range(10):
        angle = i*step

        x = x_obstacle + circling_radius*math.cos(angle)
        y = y_obstacle + circling_radius*math.sin(angle)
        # Parallell to the circumference of the circle.
        theta = angle - math.pi - (math.pi/2)

        # If a 12x12 px area unoccupied around.
        if free_space(x, y, 6):
            possible_poses.append((x, y, theta))

    # Find current pose.
    tf_listener=tf.TransformListener()
    robot_pos = [0.0, 0.0]

    found_pose = False
    
    while not found_pose:
        try:
            (trans, rot)=tf_listener.lookupTransform(
                '/map', '/base_footprint', rospy.Time(0))
            robot_pos[0]=trans[0]
            robot_pos[1]=trans[1]

            found_pose = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    # Just some high value.
    shortest_dist = 1000

    # Finding the closest capturing position from where the robot is currently at.
    for pose in possible_poses:
        dist = math.sqrt((robot_pos[0] - pose[0])**2 + (robot_pos[1] - pose[1])**2)

        if dist < shortest_dist:
            selected_pose = pose
            shortest_dist = dist

    if len(possible_poses) > 0:
        return selected_pose
    else:
        return []


def navigate_to_pose(pose):

    mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    mb_client.wait_for_server()

    mb_goal = MoveBaseGoal()
    mb_goal.target_pose.header.frame_id = "map"
    mb_goal.target_pose.header.stamp = rospy.get_rostime()
    mb_goal.target_pose.pose.position.x = pose[0]
    mb_goal.target_pose.pose.position.y = pose[1]
    mb_goal.target_pose.pose.orientation.z = math.sin(pose[2]/2)
    mb_goal.target_pose.pose.orientation.w = math.cos(pose[2]/2)

    mb_client.send_goal(mb_goal)
    mb_client.wait_for_result(rospy.Duration.from_sec(60.0))

def handle_drive_around(req):
    global obstacle_map

    x_obstacle = req.obstaclePosX
    y_obstacle = req.obstaclePosY
    circling_radius = req.circlingRadius

    pose = find_starting_pose(x_obstacle, y_obstacle, circling_radius)
    print(pose)

    if len(pose) > 0:
        navigate_to_pose(pose)

        drive_around(circling_radius)
        region = extract_region(x_obstacle, y_obstacle, circling_radius)

        if (np.count_nonzero(region) > 0):
            # Finding simplified boundary.
            boundary = find_boundary(region)

            width = int(round(20*2*circling_radius))
            height = width
            left, bottom = map_to_img(x_obstacle - circling_radius, y_obstacle - circling_radius)

            lats = []
            lngs = []

            for b in boundary:
                # Position of boundary point in the image.
                x_img = left + b[0][0]
                y_img = (bottom - height) + b[0][1]

                # On the map.
                x_map = (x_img - 199)*0.05
                y_map = (183 - y_img)*0.05

                # In (lat, lng).
                lat = y_map
                lng = x_map

                lats.append(lat)
                lngs.append(lng)

            print(lats)
            print(lngs)

        else:
            print("Empty region.")
            return DriveAroundResponse(0, [], [])

        return DriveAroundResponse(1, lats, lngs)
    else:
        print("Could not find a suitable pose to start from.")
        return DriveAroundResponse(0, [], [])


def drive_around_server():
    rospy.init_node('drive_around', anonymous=True)
    rospy.Subscriber('/eva/scan_mismatches',
                     ScanMismatches, map_callback, queue_size=1)

    s = rospy.Service('/eva/drive_around',
                      DriveAround, handle_drive_around)
    rospy.spin()


if __name__ == "__main__":
    drive_around_server()
