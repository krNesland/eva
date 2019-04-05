#!/usr/bin/env python

# Follows the requested route

# Subscribe: /tf
# Publish: /move_base/goal

import rospy
import tf
import roslib
import math

from eva_a.srv import *
from move_base_msgs.msg import MoveBaseActionGoal

# Could maybe benefit from being implemented as an action and not a server. Actions can report back during execution, not only at the end.
# Does not currently return an error if the route is cancelled.

def handle_follow_route(req):

    print("FollowRoute received a new route.")

    if len(req.latVec) < 1:
        print("A route must have at least one waypoint.")
        return FollowRouteResponse(0)
    
    if not len(req.latVec) == len(req.lngVec):
        print("xVec and yVec must have equal length.")
        return FollowRouteResponse(0)

    finished = False
    waypoints = []
    dist = 1.0

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

    listener = tf.TransformListener()
    pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)

    now_goal = waypoints.pop(0)
    now_msg = MoveBaseActionGoal()
    now_msg.goal.target_pose.header.frame_id = "map"
    now_msg.goal.target_pose.pose.position.x = now_goal[0]
    now_msg.goal.target_pose.pose.position.y = now_goal[1]
    now_msg.goal.target_pose.pose.orientation.w = 1.0

    rospy.sleep(0.5)
    pub.publish(now_msg)

    rate = rospy.Rate(10.0)
    while (not rospy.is_shutdown()) and (not finished):
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            # Current position of robot.
            now_pos = (trans[0], trans[1])
            # Distance to waypoint.
            dist = math.sqrt((now_pos[0] - now_goal[0])**2 + (now_pos[1] - now_goal[1])**2)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # If close enough to waypoint, we head towards the next waypoint.
        if dist < 0.1:
            if not len(waypoints) < 1:
                now_goal = waypoints.pop(0)
                now_msg.goal.target_pose.pose.position.x = now_goal[0]
                now_msg.goal.target_pose.pose.position.y = now_goal[1]
                # Orientation as quaternion.
                now_msg.goal.target_pose.pose.orientation.z = math.sin(now_goal[2]/2)
                now_msg.goal.target_pose.pose.orientation.w = math.cos(now_goal[2]/2)
                pub.publish(now_msg)
                print("FollowRoute is heading for next waypoint.")
            else:
                finished = True
                print("FollowRoute finished.")

        rate.sleep()


    rospy.sleep(1.0)
    return FollowRouteResponse(1) # 1 if success.

def follow_route_server():

    rospy.init_node('follow_route_server')
    print("FollowRoute service waiting for call...")
    s = rospy.Service('/eva/follow_route', FollowRoute, handle_follow_route)
    rospy.spin()

if __name__ == "__main__":
    follow_route_server()