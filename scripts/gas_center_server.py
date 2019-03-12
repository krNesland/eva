#!/usr/bin/env python

from eva_a.srv import *
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import rospy
import tf
import roslib

gasLevel = 0.0

def gasLevelCallback(data):
    global gasLevel
    gasLevel = float(data.data)
    print(gasLevel)

def handle_gas_center(req):

    print("GasCenter service called.")

    global gasLevel

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    velMsg = Twist()
    velMsg.linear.x = 0.1
    gasLevel = 20.0 # Test mode.

    if gasLevel < 10.0:
        print("Gas level is too low to start the search.")
        return GasCenterResponse(0)

    finished = False
    rate = rospy.Rate(10.0)
    while (not rospy.is_shutdown()) and (not finished):
        pub.publish(velMsg)

        rate.sleep()

    return GasCenterResponse(1) # 1 if success.

def gas_center_server():

    rospy.init_node('gas_center_server')
    rospy.Subscriber("gas_level", String, gasLevelCallback)
    print("GasCenter service waiting for call..")
    s = rospy.Service('gas_center', GasCenter, handle_gas_center)
    rospy.spin()

if __name__ == "__main__":
    gas_center_server()