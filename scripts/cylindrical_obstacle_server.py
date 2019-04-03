#!/usr/bin/env python

# Uses a Hough transformation to find the radii and center of the most likely circle.

# Subscribe: /eva/scan_mismatches

import roslib
import rospy
import numpy as np
import cv2 as cv
import math

from eva_a.msg import *
from eva_a.srv import *

# Could have had the (x, y)-dimensions of the accumulator in a different size than the image.

obstacle_map = np.zeros((384, 384), dtype=np.float32)

def callback(data):
    global obstacle_map
    obstacle_data = np.array(data.data, dtype=np.float32)
    obstacle_map = np.reshape(obstacle_data, (data.height, data.width))


def hough_circles(obstacle_map):
    try:
        resolution = rospy.get_param('resolution')
        max_r = rospy.get_param('/eva/maxCircleRadii')/resolution
        min_r = rospy.get_param('/eva/minCircleRadii')/resolution
    except:
        print("Unable to load Hough circle parameters.")
        resolution = 0.05
        max_r = 0.5/resolution
        min_r = 0.1/resolution

    # The number of different radii that are searched for.
    num_r = 8
    # Same shape of accumulator as image (except in the radius dimension).
    num_y, num_x = obstacle_map.shape
    # The number of votes per pixel and radius.
    num_theta = 360/10

    # The positions where there are pixels that should vote.
    non_zero_row, non_zero_col = np.nonzero(obstacle_map > 0.5)

    # The accumulator where the votes are stored.
    acc = np.zeros((num_y, num_x, num_r), dtype=np.float32)

    print(non_zero_row.shape)

    # Voting, (x - a)^2 + (y - b)^2 = r^2.
    for r_counter, r in enumerate(np.linspace(min_r, max_r, num_r)):
        for y, x in zip(non_zero_row, non_zero_col):
            old_centers = []
            for theta in np.linspace(-math.pi, math.pi, num_theta):
                # y coordinate of center vote.
                a = int(round(x - r*math.cos(theta)))
                # x coordinate of center vote.
                b = int(round(y - r*math.sin(theta)))

                # Don't want a single pixel to be able to vote on the same (b, a) more than once.
                if not (b, a) in old_centers:
                    # Only considering circles with center inside the image.
                    if a >= 0 and a < num_x and b >= 0 and b < num_y:
                        acc[b][a][r_counter] = acc[b][a][r_counter] + obstacle_map[y][x]

                    old_centers.append((b, a))


    # If enough votes have beed gathered.
    if np.max(acc) > 8:
        # Index of the max in a flattened version of the array.
        best_index = int(np.argmax(acc))

        # Converting to 3D indices.
        best_y_index = best_index/(num_x*num_r)
        best_index = best_index - best_y_index*num_x*num_r
        best_x_index = best_index/num_r
        best_index = best_index - best_x_index*num_r
        best_r_index = best_index

        best_radius = int((np.linspace(min_r, max_r, num_r))[best_r_index])

        '''
        cv.circle(obstacle_map, (best_x_index, best_y_index), best_radius, 100)
        cv.imshow("imgz", obstacle_map)
        cv.waitKey()
        '''

        return (1, best_x_index, best_y_index, best_radius)
    else:
        print("No cylindrical obstacles detected.")
        return (0, 0.0, 0.0, 0.0)


def handle_cylindrical_obstacles(req):
    global obstacle_map

    print("Request obtained.")

    success, x_center, y_center, radius = hough_circles(obstacle_map)

    try:
        resolution = rospy.get_param('resolution')
    except:
        print("Unable to load resolution parameter.")
        resolution = 0.05

    if success:
        lat_center = (x_center - 200)*resolution
        lng_center = (y_center - 184)*resolution
        radius = radius*resolution

        print((lat_center, lng_center, radius))

        return ReportCylindricalObstacleResponse(1, lat_center, lng_center, radius)
    else:
        return ReportCylindricalObstacleResponse(0, 0.0, 0.0, 0.0)


def cylindrical_obstacles_server():
    rospy.init_node('cylindrical_obstacles', anonymous=True)
    rospy.Subscriber('/eva/scan_mismatches',
                     ScanMismatches, callback, queue_size=1)

    s = rospy.Service('/eva/cylindrical_obstacles',
                      ReportCylindricalObstacle, handle_cylindrical_obstacles)
    rospy.spin()


if __name__ == "__main__":
    cylindrical_obstacles_server()
