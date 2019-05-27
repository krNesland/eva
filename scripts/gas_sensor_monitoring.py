#!/usr/bin/env python

# Commands the robot to move to a pre-defined position when the gas reading from the wall-mounted sensor rises above a given threshold.

# Subscribe: /eva/gas_level

import rospy
import actionlib
from std_msgs.msg import Float32
from move_base_msgs.msg import *

start_move = False

def send_navigation_command(pub):
    try:
        goal_x = rospy.get_param('/eva/fixedSensorX') - 0.5
        goal_y = rospy.get_param('/eva/fixedSensorY') + 0
    except:
        print("Fixed gas sensor position is unknown.")
        return

    mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    mb_client.wait_for_server()

    mb_goal = MoveBaseGoal()
    mb_goal.target_pose.header.frame_id = "map"
    mb_goal.target_pose.header.stamp = rospy.get_rostime()
    mb_goal.target_pose.pose.position.x = goal_x
    mb_goal.target_pose.pose.position.y = goal_y
    mb_goal.target_pose.pose.orientation.z = 0.0
    mb_goal.target_pose.pose.orientation.w = 1.0

    mb_client.send_goal(mb_goal)
    mb_client.wait_for_result(rospy.Duration.from_sec(60.0))

def callback(data):
    global start_move
    gas_level = data.data

    if gas_level > 17.7:
        start_move = True
    
def listener():
    global start_move

    rospy.init_node('gas_sensor_monitoring', anonymous=True)

    pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)

    # Waiting a little (just for visualization).
    rospy.sleep(7)

    rospy.Subscriber('/eva/fixed_gas_level', Float32, callback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Checking if the robot should start moving (if the mounted sensor has triggered an alarm).
        if start_move:
            send_navigation_command(pub)
            break
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    listener()