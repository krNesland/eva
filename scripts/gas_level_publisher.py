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

# Fixed refers to the fixed gas sensor and robot refers to the sensor on the moveable robot.

def talker():
    rospy.init_node('gas_level_publisher', anonymous=True)

    try:
        gas_cloud_x = rospy.get_param("/eva/gasX")
        gas_cloud_y = rospy.get_param("/eva/gasY")

        fixed_sensor_x = rospy.get_param("/eva/fixedSensorX")
        fixed_sensor_y = rospy.get_param("/eva/fixedSensorY")
        
    except:
        print("Not able to load gasCenter and sensorCenter.")
        
        gas_cloud_x = 3.0
        gas_cloud_y = -3.0

        fixed_sensor_x = 4.0
        fixed_sensor_y = -4.0
        
    gas_cloud_center = (gas_cloud_x, gas_cloud_y)
    fixed_sensor_center = (fixed_sensor_x, fixed_sensor_y)

    gas_cloud_state_pub = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=10)
    gas_cloud_state_msg = LinkState()
    gas_cloud_state_msg.link_name = 'gas_cloud'
    gas_cloud_state_msg.pose.position.x = gas_cloud_x
    gas_cloud_state_msg.pose.position.y = gas_cloud_y
    gas_cloud_state_msg.pose.position.z = 0.3
    gas_cloud_state_msg.pose.orientation.w = 1.0
    gas_cloud_state_msg.reference_frame = 'map'

    fixed_sensor_state_pub = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=10)
    fixed_sensor_state_msg = LinkState()
    fixed_sensor_state_msg.link_name = 'gas_sensor'
    fixed_sensor_state_msg.pose.position.x = fixed_sensor_x
    fixed_sensor_state_msg.pose.position.y = fixed_sensor_y
    fixed_sensor_state_msg.pose.position.z = 0.3
    fixed_sensor_state_msg.pose.orientation.w = 1.0
    fixed_sensor_state_msg.reference_frame = 'map'

    robot_sensor_pub = rospy.Publisher('/eva/gas_level', String, queue_size=10)
    fixed_sensor_pub = rospy.Publisher('/eva/fixed_gas_level', String, queue_size=10)

    listener = tf.TransformListener()
    rate = rospy.Rate(2) # 2hz

    while not rospy.is_shutdown():
        gas_cloud_state_pub.publish(gas_cloud_state_msg)
        fixed_sensor_state_pub.publish(fixed_sensor_state_msg)
        
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            now_pos = (trans[0], trans[1])
            robot_dist = math.sqrt((gas_cloud_center[0] - now_pos[0])**2 + (gas_cloud_center[1] - now_pos[1])**2)
            fixed_dist = math.sqrt((gas_cloud_center[0] - fixed_sensor_center[0])**2 + (gas_cloud_center[1] - fixed_sensor_center[1])**2)

            robot_level = 0.0
            robot_noise = random.gauss(0, 0.5) # Mean and Std.

            fixed_level = 0.0
            fixed_noise = random.gauss(0, 0.5)

            if robot_dist < 2.0:
                robot_level = 100*(1 - robot_dist/2)

            robot_level = robot_level + robot_noise

            if robot_level < 0.0:
                robot_level = 0.0

            if fixed_dist < 2.0:
                fixed_level = 100*(1 - fixed_dist/2)

            fixed_level = fixed_level + fixed_noise

            if fixed_level < 0.0:
                fixed_level = 0.0

            robot_gas_str = "%.1f" % robot_level
            robot_sensor_pub.publish(robot_gas_str)

            fixed_gas_str = "%.1f" % fixed_level
            fixed_sensor_pub.publish(fixed_gas_str)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass