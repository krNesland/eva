#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# bridge.cv2_to_imgmsg(cvImage, "bgr8")

bridge = CvBridge()

def callback(data):
    try:
      cvImage = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    (rows, cols, channels) = cvImage.shape
    
    cv2.imshow("Image window", cvImage)
    cv2.waitKey(3)
    
def listener():
    rospy.init_node('image_viewer', anonymous=True)
    rospy.Subscriber("camera/rgb/image_raw", Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()