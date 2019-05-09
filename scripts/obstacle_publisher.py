#!/usr/bin/env python

# Subscribe: /eva/scan_mismatches

import roslib
import rospy
import numpy as np
import cv2 as cv
import math
from os import path
import rospkg

from eva_a.msg import *

# Could have had the (x, y)-dimensions of the accumulator in a different size than the image.

obstacle_map = np.zeros((384, 384), dtype=np.uint8)
rospack = rospkg.RosPack()
outside_map = cv.imread(path.join(rospack.get_path('eva_a'), 'map', 'map_outside.pgm'), cv.IMREAD_GRAYSCALE)
outside_pixels = np.nonzero(outside_map)

class Obstacle:
    def __init__(self, cnt):
        self.cnt = cnt

        # Finding centroid.
        M = cv.moments(cnt)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        self.center = (cx, cy)

    def draw(self, canvas):
        cv.circle(canvas, self.center, 10, 255, 1)

    def get_lng(self, resolution):
        return (self.center[0] - 199)*resolution

    def get_lat(self, resolution):
        return (-self.center[1] + 183)*resolution

def find_obstacles():
    global obstacle_map
    global outside_pixels

    try:
        close_radius = rospy.get_param('/eva/obstacleCloseRadius')
        open_radius = rospy.get_param('/eva/obstacleOpenRadius')
        min_obstacle_area = rospy.get_param('/eva/minObstacleArea')
        
    except:
        print("Unable to load morphology parameters.")
        close_radius = 2
        open_radius = 3
        min_obstacle_area = 6

    # Threshold.
    ret, thresh = cv.threshold(obstacle_map, 150, 255, cv.THRESH_BINARY)
    # Removing all values outside the borders of the map.
    thresh[outside_pixels] = 0

    # Close to make more connected.
    # se_close = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2*close_radius, 2*close_radius))
    se_closed = np.array([[1, 1], [1, 1]], dtype=np.uint8)
    closed = cv.morphologyEx(thresh, cv.MORPH_CLOSE, se_closed)

    # Open to remove thin structures and small grains.
    # se_open = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2*open_radius, 2*open_radius))
    se_open = np.array([[1, 1], [1, 1]], dtype=np.uint8)
    opened = cv.morphologyEx(closed, cv.MORPH_OPEN, se_open)

    # Find contours.
    im2, contours, hierarchy = cv.findContours(opened, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    obstacles = []

    contour_img = np.zeros((384, 384), dtype=np.uint8)

    # Filtering out some of the contours that are unlikely to be obstacles.
    for cnt in contours:
        # Not interested in lines.
        if len(cnt) < 4:
            continue

        obst = Obstacle(cnt)
        obstacles.append(obst)

    cv.drawContours(contour_img, contours, -1, 255, 1)

    cv.imshow("Obstacles closed", closed)
    cv.imshow("Obstacles opened", opened)
    cv.imshow("Contours", contour_img)
    cv.waitKey(30)

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

            for obstacle in obstacle_array:
                msg.latCenters.append(obstacle.get_lat(resolution))
                msg.lngCenters.append(obstacle.get_lng(resolution))

        else:
            msg.numObstacles = 0
            msg.latCenters = []
            msg.lngCenters = []

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