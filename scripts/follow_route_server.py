#!/usr/bin/env python

from eva_a.srv import *
import rospy

# Could maybe benefit from being implemented as an action and not a server. Actions can report back during execution, not only at the end.

def handle_follow_route(req):
    print(req.xVec)
    print(req.yVec)
    return FollowRouteResponse(1) # 1 if success.

def follow_route_server():
    rospy.init_node('follow_route_server')
    s = rospy.Service('follow_route', FollowRoute, handle_follow_route)
    print "Ready to follow route."
    rospy.spin()

if __name__ == "__main__":
    follow_route_server()