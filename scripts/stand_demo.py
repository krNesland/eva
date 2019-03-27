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

bridge = CvBridge()
cv_image = np.zeros((480, 640, 3), np.uint8)

def callback(data):
    global cv_image
    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    
def objectTracking():
    global cv_image

    rospy.sleep(2.0)

    frame = cvImage
    tracker = cv2.TrackerGOTURN_create()
    bbox = cv2.selectROI(frame, False)
    ok = tracker.init(frame, bbox)

    while True:
        frame = cv_image
    
        # Start timer
        timer = cv2.getTickCount()
    
        # Update tracker
        ok, bbox = tracker.update(frame)
    
        # Calculate Frames per second (FPS)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
    
        # Draw bounding box
        if ok:
            # Tracking success
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
        else :
            # Tracking failure
            cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
    
        # Display tracker type on frame
        cv2.putText(frame, "GOTURN Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)
    
        # Display FPS on frame
        cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
    
        # Display result
        cv2.imshow("Tracking", frame)
    
        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27:
            break
    
def listener():
    rospy.init_node('stand_demo', anonymous=True)

    rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, callback)

    objectTracking()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()