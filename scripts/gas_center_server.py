#!/usr/bin/env python

from eva_a.srv import *
from std_msgs.msg import String
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

    finished = False
    rate = rospy.Rate(10.0)
    while (not rospy.is_shutdown()) and (not finished):
        try:
            pass
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

    return GasCenterResponse(1) # 1 if success.

def gas_center_server():

    rospy.init_node('gas_center_server')
    print("GasCenter service initializing...")
    rospy.Subscriber("gas_level", String, gasLevelCallback)
    s = rospy.Service('gas_center', GasCenter, handle_gas_center)
    rospy.spin()

if __name__ == "__main__":
    gas_center_server()