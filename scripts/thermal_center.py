#!/usr/bin/env python

# Subscribe: /camera/rgb/image_raw/compressed

import roslib
import rospy
import cv2 as cv
import math
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
pub = rospy.Publisher("eva/gas_image", Image, queue_size=1)

def callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        org_image = np.copy(cv_image)
    except CvBridgeError as e:
        print(e)
    
    (rows, cols, channels) = cv_image.shape

    hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

    lower_blue = np.array([100,50,50])
    upper_blue = np.array([140,255,255])

    mask = cv.inRange(hsv, lower_blue, upper_blue)

    # Vectorized version of finding the centroid.

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

    try:
        gas_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        pub.publish(gas_image)
    except CvBridgeError as e:
        print(e)  

    cv.imshow("Image window", cv_image)
    cv.imshow("Mask window", mask)
    cv.imshow("Marker window", org_image)
    cv.waitKey(30)
    
def listener():
    rospy.init_node('thermal_center', anonymous=True)

    rospy.Subscriber("/turtlebot3_camera", Image, callback, queue_size=1)   

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
