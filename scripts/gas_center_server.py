#!/usr/bin/env python

from eva_a.srv import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import rospy
import tf
import roslib
import math

gasLevel = 0.0

def gasLevelCallback(data):
    global gasLevel
    gasLevel = float(data.data)

def handle_gas_center(req):

    print("GasCenter service called.")

    global gasLevel
    gasLevelVec = [0.0, 0.0, 0.0, 0.0] # Up, down, left, right.

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    
    velMsgForward = Twist()
    velMsgForward.linear.x = 0.1

    velMsgBackward = Twist()
    velMsgBackward.linear.x = -0.1

    velMsgLeft = Twist()
    velMsgLeft.angular.z = 1.0

    velMsgRight = Twist()
    velMsgRight.angular.z = -1.0

    velMsgStop = Twist()

    # gasLevel = 20.0 # Test mode.

    if gasLevel < 10.0:
        print("Gas level is too low to start the search.")
        return GasCenterResponse(0)

    finished = False

    while (not rospy.is_shutdown()) and (not finished):
        # Up.
        print("Up")
        pub.publish(velMsgForward)
        rospy.sleep(2.0)
        gasLevelVec[0] = gasLevel

        # Down.
        print("Down")
        pub.publish(velMsgBackward)
        rospy.sleep(4.0)
        gasLevelVec[1] = gasLevel

        # Left.
        print("Left")
        pub.publish(velMsgForward)
        rospy.sleep(2.0)
        pub.publish(velMsgLeft)
        rospy.sleep(1.57)
        pub.publish(velMsgForward)
        rospy.sleep(2.0)
        gasLevelVec[2] = gasLevel

        # Right.
        print("Right")
        pub.publish(velMsgBackward)
        rospy.sleep(4.0)
        gasLevelVec[3] = gasLevel

        # Reset.
        print("Reset")
        pub.publish(velMsgForward)
        rospy.sleep(2.0)
        pub.publish(velMsgRight)
        rospy.sleep(1.57)
        pub.publish(velMsgStop)

        # Find the gradient.

        dx = gasLevelVec[3] - gasLevelVec[2]
        dy = gasLevelVec[0] - gasLevelVec[1]

        angle = math.atan2(dy, dx) - math.pi/2
        print(gasLevelVec)
        print(angle)

        # Turn towards the gradient.
        print("Turn towards gradient")
        if angle > 0:
            pub.publish(velMsgLeft)
            rospy.sleep(abs(angle))
        else:
            pub.publish(velMsgRight)
            rospy.sleep(abs(angle))

        # Move towards the gradient.
        print("Move towards gradient")
        pub.publish(velMsgForward)
        rospy.sleep(2.0)

        if gasLevel > 95:
            finished = True
            pub.publish(velMsgStop)

    print("Found gas source.")
    return GasCenterResponse(1) # 1 if success.

def gas_center_server():

    rospy.init_node('gas_center_server')
    rospy.Subscriber('/eva/gas_level', String, gasLevelCallback)
    print("GasCenter service waiting for call..")
    s = rospy.Service('gas_center', GasCenter, handle_gas_center)
    rospy.spin()

if __name__ == "__main__":
    gas_center_server()