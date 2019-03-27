#!/usr/bin/env python

# Subscribe: /camera/rgb/image_raw/compressed

import roslib
import rospy
import cv2 as cv
import math
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

EXPERIMENT = False # Experiment if real life Turtlebot is running.

bridge = CvBridge()

def callback(data):
    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    (rows, cols, channels) = cv_image.shape

    hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])

    mask = cv.inRange(hsv, lower_blue, upper_blue)

    # Vectorized version of finding the centroid.

    x_arms, y_arms = np.meshgrid(range(0, cols), range(0, rows))

    x_moments = np.multiply(x_arms, mask)
    y_moments = np.multiply(y_arms, mask)

    total = np.sum(mask)
    x_moment = np.sum(x_moments)
    y_moment = np.sum(y_moments)

    '''
    # Trying to remove outliers. Only works when there is a single blue spot one is looking for. Maybe it makes it too slow? Lots of calculations.
    if not (total == 0):
        xArm = xMoment/total
        yArm = yMoment/total

        xArm = int(math.floor(xArm))
        yArm = int(math.floor(yArm))

        sx = np.sqrt((np.sum(np.square(np.multiply(xv - xArm, mask))))/((np.sum(mask) - 1)))
        sy = np.sqrt((np.sum(np.square(np.multiply(yv - yArm, mask))))/((np.sum(mask) - 1)))

        zx = np.divide((xv - xArm), sx)
        xMask = np.abs(zx) < 2.0

        zy = np.divide((yv - yArm), sy)
        yMask = np.abs(zy) < 1.8

        xyMask = np.multiply(xMask, yMask)
        mask = np.multiply(xyMask, mask)

        xm = np.multiply(xv, mask)
        ym = np.multiply(yv, mask)

        total = np.sum(mask)
        xMoment = np.sum(xm)
        yMoment = np.sum(ym)
    '''

    if not (total == 0):
        x_arm = x_moment/total
        y_arm = y_moment/total

        x_arm = int(math.floor(x_arm))
        y_arm = int(math.floor(y_arm))

        cv.drawMarker(cv_image, (x_arm, y_arm), (0, 0, 255))
    else:
        print("No colour in the correct range was found.")

    cv.imshow("Image window", cv_image)
    cv.imshow("Mask window", mask)
    cv.waitKey(3)
    
def listener():
    rospy.init_node('thermal_center', anonymous=True)

    if EXPERIMENT:
        rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, callback, queue_size=1)
    else:
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, callback, queue_size=1) # Simulation.    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()