#!/usr/bin/env python

import roslib
import rospy
roslib.load_manifest('eva_a')
import actionlib
import math

from move_base_msgs.msg import *

def execute_and_time():
    rospy.init_node("narrow")

    rospy.sleep(8.0)

    mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    mb_client.wait_for_server()

    mb_goal = MoveBaseGoal()
    mb_goal.target_pose.header.frame_id = "map"
    mb_goal.target_pose.header.stamp = rospy.get_rostime()
    mb_goal.target_pose.pose.position.x = 3.9
    mb_goal.target_pose.pose.position.y = -1
    mb_goal.target_pose.pose.orientation.z = 0
    mb_goal.target_pose.pose.orientation.w = 1

    mb_client.send_goal(mb_goal)
    start = rospy.Time.now().to_sec()
    mb_client.wait_for_result()
    
    time = rospy.Time.now().to_sec() - start
    rospy.loginfo("Time usage: " + str(time))


if __name__ == "__main__":
    execute_and_time()
