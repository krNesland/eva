#! /usr/bin/env python

# Follows the requested route

# Subscribe: /move_base/status
# Publish: /move_base/goal

import rospy
import roslib
roslib.load_manifest('eva_a')
import actionlib
import math

from eva_a.msg import *
from move_base_msgs.msg import *

# Missing the cancel part. Have to do something like set_preempted...

class FollowRouteAction(object):
    # create messages that are used to publish feedback/result
    _feedback = eva_a.msg.FollowRouteFeedback()
    _result = eva_a.msg.FollowRouteResult()

    def __init__(self, name):
        print(name)
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, eva_a.msg.FollowRouteAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        print("Server is ready.")
      
    def execute_cb(self, goal):
        print("Starting execution.")
        self._feedback.headingFor = goal.firstWaypoint
        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, following route from waypoint %i.' % (self._action_name, goal.firstWaypoint))

        if len(goal.latVec) < 1:
            rospy.loginfo("A route must have at least one waypoint.")
            self._as.set_aborted(0, "Not enough waypoints.")
            return
        
        if not len(goal.latVec) == len(goal.lngVec):
            rospy.loginfo("xVec and yVec must have equal length.")
            self._as.set_aborted(0, "Not the same number of x and y coordiantes.")
            return

        mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        mb_client.wait_for_server()

        # Building up the list of commands.
        for i in range(self._feedback.headingFor, len(goal.latVec)):
            x = goal.lngVec[i]
            y = goal.latVec[i]
            angle = 0.0

            # If the waypoint is not the last one, we want the angle to head towards the next waypoint.
            if i < (len(goal.latVec) - 1):
                dx = goal.lngVec[i + 1] - x
                dy = goal.latVec[i + 1] - y
                angle = math.atan2(dy, dx)

            mb_goal = MoveBaseGoal()
            mb_goal.target_pose.header.frame_id = "map"
            mb_goal.target_pose.header.stamp = rospy.get_rostime()
            mb_goal.target_pose.pose.position.x = x
            mb_goal.target_pose.pose.position.y = y
            mb_goal.target_pose.pose.orientation.z = math.sin(angle/2)
            mb_goal.target_pose.pose.orientation.w = math.cos(angle/2)

            # Sends the goal to the action server.
            mb_client.send_goal(mb_goal)

            self._as.publish_feedback(self._feedback)

            # Waits for the server to finish performing the action. Can use max 60 sec to reach the waypoint.
            mb_client.wait_for_result(rospy.Duration.from_sec(60.0))

            self._feedback.headingFor = self._feedback.headingFor + 1

            # Prints out the result of executing the action (mb does not send back a result)
            # return mb_client.get_result()

        self._result.success = 1
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)
        
        '''
        # start executing the action
        for i in range(1, goal.order):
            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
            r.sleep()
        '''
          
        
if __name__ == '__main__':
    rospy.init_node('follow_route_server')
    server = FollowRouteAction(rospy.get_name())
    rospy.spin()