#!/usr/bin/env python

# Uses a Hough transformation to find the radii and center of the most likely circle.

# Subscribe: /eva/scan_mismatches

import roslib
import rospy
import numpy as np
import cv2 as cv
import math

from eva_a.msg import *

# Could have had the (x, y)-dimensions of the accumulator in a different size than the image.


def hough_circles(img):
    try:
        max_r = rospy.get_param('/eva/maxCircleRadii')
        min_r = rospy.get_param('/eva/minCircleRadii')
    except:
        print("Unable to load circle limits, defaulting to 0.1 and 0.5.")
        max_r = 0.5*20
        min_r = 0.1*20

    # The number of different radii that are searched for.
    num_r = 8
    # Same shape of accumulator as image (except in the radius dimension).
    num_y, num_x = img.shape
    # The number of votes per pixel and radius.
    num_theta = 360/10

    # The positions where there are pixels that should vote.
    non_zero_row, non_zero_col = np.nonzero(img)

    # The accumulator where the votes are stored.
    acc = np.zeros((num_y, num_x, num_r), dtype=np.float)

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
                        acc[b][a][r_counter] = acc[b][a][r_counter] + 1

                    old_centers.append((b, a))

    # If votes have beed gathered.
    if not np.max(acc) < 1:
        # Index of the max in a flattened version of the array.
        best_index = int(np.argmax(acc))

        # Converting to 3D indices.
        best_y_index = best_index/(num_x*num_r)
        best_index = best_index - best_y_index*num_x*num_r
        best_x_index = best_index/num_r
        best_index = best_index - best_x_index*num_r
        best_r_index = best_index

        best_radius = int((np.linspace(min_r, max_r, num_r))[best_r_index])

        cv.circle(img, (best_x_index, best_y_index), best_radius, 100)
        cv.imshow("img", img)
        cv.waitKey(30)
    else:
        print("No votes received.")


def callback(data):
    img_data = np.array(data.data, dtype=np.uint8)
    img = np.reshape(img_data, (data.height, data.width))

    img[img > 0] = 255

    hough_circles(img)


def listener():
    rospy.init_node('cylindrical_obstacles', anonymous=True)

    rospy.Subscriber('/eva/scan_mismatches',
                     ScanMismatches, callback, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
