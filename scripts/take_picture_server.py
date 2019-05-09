#!/usr/bin/env python

# Takes a picture of an obstacle given its position.

# Subscribe: 

import roslib
import rospy
import numpy as np
import cv2 as cv
import math
import tf
import rospkg
from os import path

from nav_msgs.msg import OccupancyGrid
from eva_a.srv import *
from move_base_msgs.msg import MoveBaseActionGoal
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatusArray


map_data = np.zeros((384, 384), dtype=np.int8)
seq_id = 0

def map_callback(data):
    global map_data

    map_data = np.flipud(np.reshape(data.data, (data.info.height, data.info.width)).astype(np.int8))


# Converting from map coordinates to coordinates of map_data.
def map_to_img(x, y):
    u = int(round(20*x + 200)) - 1
    v = int(round(-20*y + 184)) - 1

    return (u, v)


def free_space(x, y, region_size):
    global map_data
    u, v = map_to_img(x, y)

    left = u - region_size
    if left < 0:
        return False

    right = u + region_size
    if right > map_data.shape[1]:
        return False

    top = v - region_size
    if top < 0:
        return False

    bottom = v + region_size
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

        # If a 12x12 px area unoccupied around.
        if free_space(x, y, 6):
            possible_poses.append((x, y, theta))


    '''

    visualizer = np.zeros((384, 384), dtype=np.uint8)
    visualizer[map_data > 50] = 50

    obstacleX, obstacleY = map_to_img(x_obstacle, y_obstacle)
    visualizer[obstacleY][obstacleX] = 250

    for tup in possible_poses:
        vizX, vizY = map_to_img(tup[0], tup[1])
        visualizer[vizY][vizX] = 150

    cv.imshow("vizz", visualizer)
    cv.waitKey()

    '''

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
    global seq_id

    pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
    msg = MoveBaseActionGoal()
    msg.goal.target_pose.header.frame_id = "map"
    msg.goal_id.id = "take_picture_goal_" + str(seq_id)
    msg.goal.target_pose.pose.position.x = pose[0]
    msg.goal.target_pose.pose.position.y = pose[1]
    # Orientation as a quaternion.
    msg.goal.target_pose.pose.orientation.z = math.sin(pose[2]/2)
    msg.goal.target_pose.pose.orientation.w = math.cos(pose[2]/2)

    rospy.sleep(0.5)
    pub.publish(msg)

    reached_goal = False
    rate = rospy.Rate(2.0)

    while (not rospy.is_shutdown()) and (not reached_goal):

        data = rospy.wait_for_message("move_base/status", GoalStatusArray)

        for s in data.status_list:
            if "take_picture_goal_" + str(seq_id) in s.goal_id.id:
                if s.status == 3:
                    reached_goal = True

        rate.sleep()


    seq_id = seq_id + 1

def handle_take_picture(req):
    global map_data

    x_obstacle = req.obstaclePosX
    y_obstacle = req.obstaclePosY

    pose = find_capturing_pose(x_obstacle, y_obstacle)
    print(pose)

    if len(pose) > 0:
        navigate_to_pose(pose)

        # Several times in order to force the newest image.
        print("Waiting for image.")
        data = rospy.wait_for_message("/turtlebot3_camera", Image)
        data = rospy.wait_for_message("/turtlebot3_camera", Image)
        data = rospy.wait_for_message("/turtlebot3_camera", Image)
        print("Image gotten.")

        bridge = CvBridge()
        rospack = rospkg.RosPack()
        rospy.sleep(0.2)
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        img_path = path.join(rospack.get_path('eva_a'), 'img', 'captures', 'capture.jpg')
        cv.imwrite(img_path, cv_image)
        print("Image stored: " + img_path)
        
        # Give it some time to store the image.
        rospy.sleep(1.0)

        return TakePictureResponse(1)
    else:
        print("Could not find a suitable pose to take a picture from.")
        return TakePictureResponse(0)


def take_picture_server():
    rospy.init_node('take_picture', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, map_callback, queue_size=1)

    s = rospy.Service('/eva/take_picture',
                      TakePicture, handle_take_picture)
    rospy.spin()


if __name__ == "__main__":
    take_picture_server()
