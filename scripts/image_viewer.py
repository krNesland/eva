#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv
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

    cv.imshow("Image window", cvImage)
    cv.waitKey(3)
    
def listener():
    rospy.init_node('image_viewer', anonymous=True)

    if EXPERIMENT:
        rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, callback)
    else:
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, callback) # Simulation.    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()