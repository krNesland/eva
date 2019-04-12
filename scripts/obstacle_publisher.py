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
        return (self.center[0] - 200)*resolution

    def get_lng(self, resolution):
        return (self.center[1] - 184)*resolution

    def get_radius(self, resolution):
        return self.radius*resolution

def find_obstacles():
    global obstacle_map

    # Threshold.
    ret, thresh = cv.threshold(obstacle_map, 150, 255, cv.THRESH_BINARY)

    # Close to make more connected.
    se_close = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
    closed = cv.morphologyEx(thresh, cv.MORPH_CLOSE, se_close)

    # Open to remove noise.
    #se_open = cv.getStructuringElement(cv.MORPH_ELLIPSE, (3, 3))
    #opened = cv.morphologyEx(closed, cv.MORPH_OPEN, se_open)

    # Find contours.
    im2, contours, hierarchy = cv.findContours(closed, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    obstacles = []

    for cnt in contours:
        if cv.contourArea(cnt) < 5:
            continue

        ellipse = cv.fitEllipse(cnt)

        # Multiplying the length of the two main axes.
        if ellipse[1][0]*ellipse[1][1] < 4:
            continue

        # If way too large for an obstacle.
        if ellipse[1][0] > 20 or ellipse[1][1] > 20:
            continue

        obst = Obstacle(cnt)
        obstacles.append(obst)

    return obstacles



def talker():
    pub = rospy.Publisher('/eva/obstacles', Obstacles, queue_size=10)

    try:
        resolution = rospy.get_param('resolution')
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