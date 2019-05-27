#! /usr/bin/env python

# Follows the route requested by the browser by communicating with move_base.

import rospy
import roslib
roslib.load_manifest('eva_a')
import actionlib
import math

from eva_a.msg import *
from move_base_msgs.msg import *

class FollowRouteAction(object):
    # Create messages that are used to publish feedback/result
    _feedback = eva_a.msg.FollowRouteFeedback()
    _result = eva_a.msg.FollowRouteResult()

    def __init__(self, name):
        print(name)
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, eva_a.msg.FollowRouteAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        print("Server is ready.")
      
    def execute_cb(self, goal):
        success = 1

        print("Starting execution.")
        self._feedback.headingFor = goal.firstWaypoint
        
        # Publish info to the console for the user.
        rospy.loginfo('%s: Executing, following route from waypoint %i.' % (self._action_name, goal.firstWaypoint))

        # Controlling the input.
        if len(goal.latVec) < 1:
            rospy.loginfo("A route must have at least one waypoint.")
            self._as.set_aborted(0, "Not enough waypoints.")
            return
        
        # Controlling the input.
        if not len(goal.latVec) == len(goal.lngVec):
            rospy.loginfo("xVec and yVec must have equal length.")
            self._as.set_aborted(0, "Not the same number of x and y coordiantes.")
            return

        # Initializing the client that will communicate with move_base.
        mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        mb_client.wait_for_server()

        # Looping through the checkpoints.
        for i in range(self._feedback.headingFor, len(goal.latVec)):
            x = goal.lngVec[i]
            y = goal.latVec[i]
            angle = 0.0

            # If the waypoint is not the last one, we want the angle to head towards the next waypoint.
            if i < (len(goal.latVec) - 1):
                dx = goal.lngVec[i + 1] - x
                dy = goal.latVec[i + 1] - y
                angle = math.atan2(dy, dx)

            # Putting together the goal that will be sent to move_base.
            mb_goal = MoveBaseGoal()
            mb_goal.target_pose.header.frame_id = "map"
            mb_goal.target_pose.header.stamp = rospy.get_rostime()
            mb_goal.target_pose.pose.position.x = x
            mb_goal.target_pose.pose.position.y = y
            mb_goal.target_pose.pose.orientation.z = math.sin(angle/2)
            mb_goal.target_pose.pose.orientation.w = math.cos(angle/2)

            # Sends the goal to the action server (move_base).
            mb_client.send_goal(mb_goal)

            # Publishing feedback to the browser on which waypoint the robot is currently chasing.
            self._as.publish_feedback(self._feedback)

            # Waits for the server to finish performing the action. Can use max 60 sec to reach the waypoint. Timeout after this.
            mb_client.wait_for_result(rospy.Duration.from_sec(60.0))

            # At this point the goal has either been reached or has been cancelled.

            # Incrementing which waypoint that is being chased.
            self._feedback.headingFor = self._feedback.headingFor + 1

            # If the route following has been cancelled from the browser.
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = 0
                # Jumping out of the waypoint-loop.
                break

        self._result.success = success
        # Reporting the result to the browser. Either success or not.
        self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('follow_route_server')
    server = FollowRouteAction(rospy.get_name())
    rospy.spin()