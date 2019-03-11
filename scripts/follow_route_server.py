#!/usr/bin/env python

from eva_a.srv import *
import rospy
import tf
import roslib
from move_base_msgs.msg import MoveBaseActionGoal
import math

# Could maybe benefit from being implemented as an action and not a server. Actions can report back during execution, not only at the end.
# Does not currently return an error if the route is cancelled.

def handle_follow_route(req):

    print("New route acquired.")

    if len(req.latVec) < 1:
        print("A route must have at least one waypoint.")
        return FollowRouteResponse(0)
    
    if not len(req.latVec) == len(req.lngVec):
        print("xVec and yVec must have equal length.")
        return FollowRouteResponse(0)

    finished = False
    waypoints = []
    dist = 1.0 # Initialization.

    for i in range(len(req.latVec)):
        x = req.latVec[i]
        y = -req.lngVec[i]
        angle = 0.0

        if i < (len(req.latVec) - 1):
            dx = req.latVec[i + 1] - x
            dy = -req.lngVec[i + 1] - y
            angle = math.atan2(dy, dx)

        waypoints.append((x, y, angle))

    listener = tf.TransformListener()
    pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)

    nowGoal = waypoints.pop(0)
    nowMsg = MoveBaseActionGoal()
    nowMsg.goal.target_pose.header.frame_id = "map"
    nowMsg.goal.target_pose.pose.position.x = nowGoal[0]
    nowMsg.goal.target_pose.pose.position.y = nowGoal[1]
    nowMsg.goal.target_pose.pose.orientation.w = 1.0
    pub.publish(nowMsg)

    rate = rospy.Rate(10.0)
    while (not rospy.is_shutdown()) and (not finished):
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            nowPos = (trans[0], trans[1])
            dist = math.sqrt((nowPos[0] - nowGoal[0])**2 + (nowPos[1] - nowGoal[1])**2)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


        if dist < 0.1:
            if not len(waypoints) < 1:
                nowGoal = waypoints.pop(0)
                nowMsg.goal.target_pose.pose.position.x = nowGoal[0]
                nowMsg.goal.target_pose.pose.position.y = nowGoal[1]
                nowMsg.goal.target_pose.pose.orientation.z = nowGoal[2]
                pub.publish(nowMsg)
                print("Heading for next waypoint.")
            else:
                finished = True
                print("Finished.")

        rate.sleep()


    return FollowRouteResponse(1) # 1 if success.

def follow_route_server():

    rospy.init_node('follow_route_server')
    print("FollowRoute service initializing...")
    s = rospy.Service('follow_route', FollowRoute, handle_follow_route)
    rospy.spin()

if __name__ == "__main__":
    follow_route_server()