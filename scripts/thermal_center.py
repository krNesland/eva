#!/usr/bin/env python

# Aims to find the center of the gas leakage (represented by the colour blue).

# Subscribe: /turtlebot3_camera
# Publish: /eva/gas_image

import roslib
import rospy
import cv2 as cv
import math
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
pub = rospy.Publisher("eva/gas_image", Image, queue_size=1)

def callback(data):
    # Converting from image message to OpenCV image.
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        org_image = np.copy(cv_image)
    except CvBridgeError as e:
        print(e)
    
    (rows, cols, channels) = cv_image.shape

    # Converting to the hsv colour space.
    hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

    # Defining the colour region that spans the blue colour we are looking for.
    lower_blue = np.array([100,50,50])
    upper_blue = np.array([140,255,255])

    # Getting a binary image with 1-pixels where the correct colour is found.
    mask = cv.inRange(hsv, lower_blue, upper_blue)

    # Calculation of the centroid starts here.

    x_arms, y_arms = np.meshgrid(range(0, cols), range(0, rows))

    x_moments = np.multiply(x_arms, mask)
    y_moments = np.multiply(y_arms, mask)

    total = np.sum(mask)
    x_moment = np.sum(x_moments)
    y_moment = np.sum(y_moments)

    if not (total < 25):
        x_arm = x_moment/total
        y_arm = y_moment/total

        x_arm = int(math.floor(x_arm))
        y_arm = int(math.floor(y_arm))

        cv.drawMarker(cv_image, (x_arm, y_arm), (0, 0, 255))
    else:
        print("Not enough pixels in the correct range was found.")

    # Publishing the result.
    try:
        gas_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        pub.publish(gas_image)
    except CvBridgeError as e:
        print(e)  
    
def listener():
    rospy.init_node('thermal_center', anonymous=True)

    rospy.Subscriber("/turtlebot3_camera", Image, callback, queue_size=1)   

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
