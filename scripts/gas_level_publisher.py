#!/usr/bin/env python

# Publishing a gas level based on how close it is to a pre-defined gas cloud.

# Subscribe: /tf
# Publish: /eva/gas_level, /eva/fixed_gas_level

import rospy
import tf
import roslib
import math
import random
import numpy as np

from std_msgs.msg import Float32

# Note that we are now using the estimated position and not the actual position in Gazebo.
# Fixed refers to the fixed gas sensor and robot refers to the sensor on the moveable robot.

def talker():
    rospy.init_node('gas_level_publisher', anonymous=True)

    # Finding the position of the gas leakage and the mounted gas sensor.
    try:
        gas_cloud_x = rospy.get_param("/eva/gasX")
        gas_cloud_y = rospy.get_param("/eva/gasY")

        fixed_sensor_x = rospy.get_param("/eva/fixedSensorX")
        fixed_sensor_y = rospy.get_param("/eva/fixedSensorY")
        
    except:
        print("Not able to load gasCenter and sensorCenter.")
        
        gas_cloud_x = 1.0
        gas_cloud_y = -1.5

        fixed_sensor_x = 0.15
        fixed_sensor_y = 0.84
    
    robot_sensor_pub = rospy.Publisher('/eva/gas_level', Float32, queue_size=10)
    fixed_sensor_pub = rospy.Publisher('/eva/fixed_gas_level', Float32, queue_size=10)
    
    gas_cloud_center = (gas_cloud_x, gas_cloud_y)
    fixed_sensor_center = (fixed_sensor_x, fixed_sensor_y)

    listener = tf.TransformListener()
    rate = rospy.Rate(2) # 2hz

    while not rospy.is_shutdown():
        
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            now_pos = (trans[0], trans[1])
            robot_dist = math.sqrt((gas_cloud_center[0] - now_pos[0])**2 + (gas_cloud_center[1] - now_pos[1])**2)
            fixed_dist = math.sqrt((gas_cloud_center[0] - fixed_sensor_center[0])**2 + (gas_cloud_center[1] - fixed_sensor_center[1])**2)

            robot_level = 0.0
            # Adding noise with a given mean and std.
            robot_noise = random.gauss(0, 0.5)

            fixed_level = 0.0
            fixed_noise = random.gauss(0, 0.5)

            # Should be zero if outside 2 m.
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

            # Rounding the final result and publishing.
            robot_sensor_pub.publish(np.around(robot_level, decimals=1))
            fixed_sensor_pub.publish(np.around(fixed_level, decimals=1))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
