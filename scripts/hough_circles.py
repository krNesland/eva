#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import math
import numpy as np

# bridge.cv2_to_imgmsg(cvImage, "bgr8") if one would like to send a cv image as a message.

EXPERIMENT = False # Experiment if real life Turtlebot is running.

bridge = CvBridge()

def callback(data):
    try:
        cvImage = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    (rows, cols, channels) = cvImage.shape
    grayImage = cv2.cvtColor(cvImage, cv2.COLOR_BGR2GRAY)

    circles = cv2.HoughCircles(grayImage, cv2.HOUGH_GRADIENT, 1, 50, param1=50, param2=30, minRadius=0, maxRadius=200)

    try:
        numTopCircles = len(circles[0][0])
        if numTopCircles > 5: # Maximum visualizing five circles.
            numTopCircles = 5

        topCircles = np.uint16(np.around(circles[0][0:(numTopCircles - 1)]))
        
        try:
            for c in topCircles:
                cv2.circle(cvImage, (c[0], c[1]), c[2], (0, 255, 0), 2)
        except:
            print("Circle out of bounds.")
    except:
        print("No circles found.")    

    cv2.imshow("Image window", cvImage)
    cv2.waitKey(3)
    
def listener():
    rospy.init_node('hough_circles', anonymous=True)

    if EXPERIMENT:
        rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, callback)
    else:
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, callback) # Simulation.    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()