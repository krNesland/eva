#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from gazebo_msgs.msg import LinkState
import tf
import roslib
import math
import random

# Note that we are now sing the estimated position and not the actual position in Gazebo.
# Also sets the position of the gas cloud in Gazebo.

def talker():
    rospy.init_node('gas_level_publisher', anonymous=True)

    try:
        gasX = rospy.get_param("gasX")
        gasY = rospy.get_param("gasY")

        gasCenter = (gasX, gasY)

        print(gasCenter)  
    except:
        print("Not able to load gasCenter. Defaulting to (3.0, -3.0).")
        gasCenter = (3.0, -3.0)

    gasPub = rospy.Publisher('gazebo/set_link_state', LinkState, queue_size=10)
    gasStateMsg = LinkState()
    gasStateMsg.link_name = "gas_cloud"
    gasStateMsg.pose.position.x = gasX
    gasStateMsg.pose.position.y = gasY
    gasStateMsg.pose.position.z = 0.3
    gasStateMsg.pose.orientation.w = 1.0
    gasStateMsg.reference_frame = "map"

    pub = rospy.Publisher('gas_level', String, queue_size=10)
    listener = tf.TransformListener()
    rate = rospy.Rate(2) # 2hz
    while not rospy.is_shutdown():
        gasPub.publish(gasStateMsg)
        
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
        talker()
    except rospy.ROSInterruptException:
        pass