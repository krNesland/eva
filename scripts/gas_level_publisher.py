#!/usr/bin/env python

# Publishing a gas level based on how close it is to a pre-defined gas cloud.

# Subscribe: /tf, 
# Publish: /eva/gas_level, /gazebo/set_link_state

import rospy
import tf
import roslib
import math
import random

from std_msgs.msg import String
from gazebo_msgs.msg import LinkState

# Note that we are now using the estimated position and not the actual position in Gazebo.
# Also sets the position of the gas cloud in Gazebo.

def talker():
    rospy.init_node('gas_level_publisher', anonymous=True)

    try:
        gas_x = rospy.get_param("/eva/gasX")
        gas_y = rospy.get_param("/eva/gasY")

        gas_center = (gas_x, gas_y)

        print(gas_center)  
    except:
        print("Not able to load gasCenter. Defaulting to (3.0, -3.0).")
        
        gas_x = 3.0
        gas_y = -3.0
        
        gas_center = (gas_x, gas_y)

    gas_pub = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=10)
    gas_state_msg = LinkState()
    gas_state_msg.link_name = 'gas_cloud'
    gas_state_msg.pose.position.x = gas_x
    gas_state_msg.pose.position.y = gas_y
    gas_state_msg.pose.position.z = 0.3
    gas_state_msg.pose.orientation.w = 1.0
    gas_state_msg.reference_frame = 'map'

    pub = rospy.Publisher('/eva/gas_level', String, queue_size=10)
    listener = tf.TransformListener()
    rate = rospy.Rate(2) # 2hz
    while not rospy.is_shutdown():
        gas_pub.publish(gas_state_msg)
        
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            now_pos = (trans[0], trans[1])
            dist = math.sqrt((gas_center[0] - now_pos[0])**2 + (gas_center[1] - now_pos[1])**2)

            level = 0.0
            noise = random.gauss(0, 0.5) # Mean and Std.

            if dist < 2.0:
                level = 100*(1 - dist/2)

            level = level + noise

            if level < 0.0:
                level = 0.0

            gas_str = "%.1f" % level
            pub.publish(gas_str)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass