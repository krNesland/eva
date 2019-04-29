#!/usr/bin/env python

# Follows the requested route

# Subscribe: /move_base/status
# Publish: /move_base/goal

import rospy
import roslib
import math

from eva_a.srv import *
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray, GoalID
from std_msgs.msg import UInt32

# Could maybe benefit from being implemented as an action and not a server. Actions can report back during execution, not only at the end.
# Does not currently return an error if the route is cancelled.

def handle_follow_route(req):
    reached_goal = False
    seq_id = 0

    print("FollowRoute received a new route.")

    if len(req.latVec) < 1:
        print("A route must have at least one waypoint.")
        return FollowRouteResponse(0)
    
    if not len(req.latVec) == len(req.lngVec):
        print("xVec and yVec must have equal length.")
        return FollowRouteResponse(0)

    finished = False
    waypoints = []

    # Building up the list of commands.
    for i in range(len(req.latVec)):
        x = req.latVec[i]
        y = -req.lngVec[i]
        angle = 0.0

        # If the waypoint is not the last one, we want the angle to head towards the next waypoint.
        if i < (len(req.latVec) - 1):
            dx = req.latVec[i + 1] - x
            dy = -req.lngVec[i + 1] - y
            angle = math.atan2(dy, dx)

        waypoints.append((x, y, angle))

    waypoint_pub = rospy.Publisher('/eva/curr_waypoint', UInt32, queue_size=1)

    pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=1)

    now_goal = waypoints.pop(0)
    now_msg = MoveBaseActionGoal()
    now_msg.goal.target_pose.header.frame_id = "map"
    now_msg.goal_id.id = "follow_route_goal_" + str(seq_id)
    now_msg.goal.target_pose.pose.position.x = now_goal[0]
    now_msg.goal.target_pose.pose.position.y = now_goal[1]
    now_msg.goal.target_pose.pose.orientation.z = math.sin(now_goal[2]/2)
    now_msg.goal.target_pose.pose.orientation.w = math.cos(now_goal[2]/2)

    rospy.sleep(0.5)
    pub.publish(now_msg)
    pub.publish(now_msg)

    while (not rospy.is_shutdown()) and (not finished):

        data = rospy.wait_for_message("move_base/status", GoalStatusArray)
        print(seq_id)

        for s in data.status_list:
            if "follow_route_goal_" + str(seq_id) in s.goal_id.id:
                if s.status == 6:
                    print("Follow route was cancelled.")
                    return FollowRouteResponse(0)

                if s.status == 3:
                    reached_goal = True
                else:
                    reached_goal = False

        if reached_goal:
            if not len(waypoints) < 1:
                seq_id = seq_id + 1

                now_goal = waypoints.pop(0)
                now_msg = MoveBaseActionGoal()
                now_msg.goal.target_pose.header.frame_id = "map"
                now_msg.goal_id.id = "follow_route_goal_" + str(seq_id)
                now_msg.goal.target_pose.pose.position.x = now_goal[0]
                now_msg.goal.target_pose.pose.position.y = now_goal[1]
                # Orientation as quaternion.
                now_msg.goal.target_pose.pose.orientation.z = math.sin(now_goal[2]/2)
                now_msg.goal.target_pose.pose.orientation.w = math.cos(now_goal[2]/2)
                rospy.sleep(0.5)
                pub.publish(now_msg)
                pub.publish(now_msg)
                ready_for_next = False
                print("FollowRoute is heading for next waypoint.")

                waypoint_pub.publish(seq_id)

                # Giving it some time to publish the new goal before checking the status.
                rospy.sleep(2.0)
            else:
                finished = True
                print("FollowRoute finished.")
                seq_id = seq_id + 1

    rospy.sleep(1.0)
    return FollowRouteResponse(1) # 1 if success.

def follow_route_server():

    rospy.init_node('follow_route_server')
    print("FollowRoute service waiting for call...")
    s = rospy.Service('/eva/follow_route', FollowRoute, handle_follow_route)
    rospy.spin()

if __name__ == "__main__":
    follow_route_server()