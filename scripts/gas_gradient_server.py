#!/usr/bin/env python

# Reads the gas level and tries to move the robot towards the gas gradient.

# Subscribe: /eva/gas_level
# Publish: /cmd_vel

import rospy
import tf
import roslib
import math

from eva_a.srv import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist

gas_level = 0.0

def gas_level_callback(data):
    global gas_level
    gas_level = float(data.data)

def handle_gas_gradient(req):

    print("GasGradient service called.")

    global gas_level
    gas_level_vec = [0.0, 0.0, 0.0, 0.0] # Up, down, left, right.

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    vel_msg_forward = Twist()
    vel_msg_forward.linear.x = 0.1

    vel_msg_backward = Twist()
    vel_msg_backward.linear.x = -0.1

    vel_msg_left = Twist()
    vel_msg_left.angular.z = 1.0

    vel_msg_right = Twist()
    vel_msg_right.angular.z = -1.0

    vel_msg_stop = Twist()

    if gas_level < 10.0:
        print("Gas level is too low to start the search.")
        return GasGradientResponse(0)

    finished = False

    while (not rospy.is_shutdown()) and (not finished):
        # Up.
        print("Up")
        pub.publish(vel_msg_forward)
        # Letting it move for 2 seconds.
        rospy.sleep(2.0)
        gas_level_vec[0] = gas_level

        # Down.
        print("Down")
        pub.publish(vel_msg_backward)
        rospy.sleep(4.0)
        gas_level_vec[1] = gas_level

        # Left.
        print("Left")
        pub.publish(vel_msg_forward)
        rospy.sleep(2.0)
        pub.publish(vel_msg_left)
        rospy.sleep(1.57)
        pub.publish(vel_msg_forward)
        rospy.sleep(2.0)
        gas_level_vec[2] = gas_level

        # Right.
        print("Right")
        pub.publish(vel_msg_backward)
        rospy.sleep(4.0)
        gas_level_vec[3] = gas_level

        # Reset.
        print("Reset")
        pub.publish(vel_msg_forward)
        rospy.sleep(2.0)
        pub.publish(vel_msg_right)
        rospy.sleep(1.57)
        pub.publish(vel_msg_stop)

        # Find the gradient.
        dx = gas_level_vec[3] - gas_level_vec[2]
        dy = gas_level_vec[0] - gas_level_vec[1]

        angle = math.atan2(dy, dx) - math.pi/2
        print(gas_level_vec)
        print(angle)

        # Turn towards the gradient.
        print("Turn towards gradient")
        if angle > 0:
            pub.publish(vel_msg_left)
            rospy.sleep(abs(angle))
        else:
            pub.publish(vel_msg_right)
            rospy.sleep(abs(angle))

        # Move towards the gradient.
        print("Move towards gradient")
        pub.publish(vel_msg_forward)
        rospy.sleep(2.0)

        if gas_level > 95:
            finished = True
            pub.publish(vel_msg_stop)

    print("Found gas source.")
    return GasGradientResponse(1) # 1 if success.

def gas_gradient_server():

    rospy.init_node('gas_gradient_server')
    rospy.Subscriber('/eva/gas_level', String, gas_level_callback)
    print("GasGradient service waiting for call..")
    s = rospy.Service('/eva/gas_gradient', GasGradient, handle_gas_gradient)
    rospy.spin()

if __name__ == "__main__":
    gas_gradient_server()