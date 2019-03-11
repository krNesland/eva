#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import tf
import roslib
import math
import random

# Note that we are now sing the estimated position and not the actual position in Gazebo.

def talker():
    gasCenter = (1.9, -2.375)

    pub = rospy.Publisher('gas_level', String, queue_size=10)
    rospy.init_node('gas_level_publisher', anonymous=True)
    listener = tf.TransformListener()
    rate = rospy.Rate(2) # 2hz
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            nowPos = (trans[0], trans[1])
            dist = math.sqrt((gasCenter[0] - nowPos[0])**2 + (gasCenter[1] - nowPos[1])**2)

            level = 0.0
            noise = random.gauss(0, 0.5) # Mean and Std.

            if dist < 2.0:
                level = 100*(1 - dist/2)

            level = level + noise

            if level < 0.0:
                level = 0.0

            gasStr = "%.1f" % level
            pub.publish(gasStr)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == '__main__':
    try:
        print("Gas level publisher initializing...")
        talker()
    except rospy.ROSInterruptException:
        pass