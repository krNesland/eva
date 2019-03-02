#!/usr/bin/env python

from eva_a.srv import *
import rospy
import tf
import roslib
from move_base_msgs.msg import MoveBaseActionGoal
import math

# Could maybe benefit from being implemented as an action and not a server. Actions can report back during execution, not only at the end.

def handle_follow_route(req):

    print("New route acquired.")

    if len(req.xVec) < 1:
        print("A route must have at least one waypoint.")
        return FollowRouteResponse(0)
    
    if not len(req.xVec) == len(req.yVec):
        print("xVec and yVec must have equal length.")
        return FollowRouteResponse(0)

    finished = False
    waypoints = []

    for i in range(len(req.xVec)):
        waypoints.append((req.xVec[i], req.yVec[i]))

    listener = tf.TransformListener()
    pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)

    nowGoal = waypoints.pop(0)
    nowMsg = MoveBaseActionGoal()
    nowMsg.goal.target_pose.header.frame_id = "odom"
    nowMsg.goal.target_pose.pose.position.x = nowGoal[0]
    nowMsg.goal.target_pose.pose.position.y = nowGoal[1]
    nowMsg.goal.target_pose.pose.orientation.w = 1.0

    rate = rospy.Rate(10.0)
    while (not rospy.is_shutdown()) and (not finished):
        try:
            (trans,rot) = listener.lookupTransform('/base_footprint', '/odom', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        nowPos = (-trans[0], -trans[1])
        dist = math.sqrt((nowPos[0] - nowGoal[0])**2 + (nowPos[1] - nowGoal[1])**2)

        if dist < 0.2:
            if not len(waypoints) < 1:
                nowGoal = waypoints.pop(0)
                nowMsg.goal.target_pose.pose.position.x = nowGoal[0]
                nowMsg.goal.target_pose.pose.position.y = nowGoal[1]
                print("Heading for next waypoint.")
            else:
                finished = True
                print("Finished.")

        pub.publish(nowMsg) # Maybe unecessary to have here?
        rate.sleep()


    return FollowRouteResponse(1) # 1 if success.

def follow_route_server():

    rospy.init_node('follow_route_server')
    print("FollowRoute service initializing...")
    s = rospy.Service('follow_route', FollowRoute, handle_follow_route)
    rospy.spin()

if __name__ == "__main__":
    follow_route_server()