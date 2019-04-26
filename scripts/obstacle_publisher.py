#!/usr/bin/env python

# Subscribe: /eva/scan_mismatches

import roslib
import rospy
import numpy as np
import cv2 as cv
import math

from eva_a.msg import *

# Could have had the (x, y)-dimensions of the accumulator in a different size than the image.

obstacle_map = np.zeros((384, 384), dtype=np.uint8)

class Obstacle:
    def __init__(self, cnt):
        self.cnt = cnt
        (x, y), radius = cv.minEnclosingCircle(cnt)
        self.center = (int(x), int(y))
        self.radius = int(radius)

    def draw(self, canvas):
        cv.circle(canvas, self.center, self.radius, 255, 1)

    def get_lat(self, resolution):
        return (self.center[0] - 199)*resolution

    def get_lng(self, resolution):
        return (self.center[1] - 183)*resolution

    def get_radius(self, resolution):
        return self.radius*resolution

def find_obstacles():
    global obstacle_map

    try:
        close_radius = rospy.get_param('/eva/obstacleCloseRadius')
        open_radius = rospy.get_param('/eva/obstacleOpenRadius')
        min_obstacle_area = rospy.get_param('/eva/minObstacleArea')
        
    except:
        print("Unable to load morphology parameters.")
        close_radius = 5
        open_radius = 2
        min_obstacle_area = 5

    # Threshold.
    ret, thresh = cv.threshold(obstacle_map, 150, 255, cv.THRESH_BINARY)

    # Close to make more connected.
    se_close = cv.getStructuringElement(cv.MORPH_ELLIPSE, (close_radius, close_radius))
    closed = cv.morphologyEx(thresh, cv.MORPH_CLOSE, se_close)

    # Open to remove thin structures and small grains.
    se_open = cv.getStructuringElement(cv.MORPH_ELLIPSE, (open_radius, open_radius))
    opened = cv.morphologyEx(closed, cv.MORPH_OPEN, se_open)

    # Find contours.
    im2, contours, hierarchy = cv.findContours(opened, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    obstacles = []

    # Filtering out some of the contours that are unlikely to be obstacles.
    for cnt in contours:
        # Not interested in lines.
        if len(cnt) < 4:
            continue

        obst = Obstacle(cnt)
        obstacles.append(obst)

    return obstacles


def talker():
    pub = rospy.Publisher('/eva/obstacles', Obstacles, queue_size=10)

    try:
        resolution = rospy.get_param('/eva/resolution')
    except:
        resolution = 0.05

    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        msg = Obstacles()
        obstacle_array = find_obstacles()

        if len(obstacle_array) > 0:
            print("Num obstacles: " + str(len(obstacle_array)))
            msg.numObstacles = len(obstacle_array)
            msg.latCenters = []
            msg.lngCenters = []
            msg.radii = []

            for obstacle in obstacle_array:
                msg.latCenters.append(obstacle.get_lat(resolution))
                msg.lngCenters.append(obstacle.get_lng(resolution))
                msg.radii.append(obstacle.get_radius(resolution))

        else:
            msg.numObstacles = 0
            msg.latCenters = []
            msg.lngCenters = []
            msg.radii = []

        pub.publish(msg)
        rate.sleep()

def callback(data):
    global obstacle_map
    obstacle_data = np.array(data.data, dtype=np.uint8)
    obstacle_map = np.reshape(obstacle_data, (data.height, data.width))
    
def listener():
    rospy.init_node('obstacle_publisher', anonymous=True)

    rospy.Subscriber('/eva/scan_mismatches',
                     ScanMismatches, callback, queue_size=1)

    talker()

    rospy.spin()

if __name__ == '__main__':
    listener()