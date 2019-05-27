#!/usr/bin/env python

# Publishing the pose of the robot such that Gazebo can position the robot correctly.

# Subscribe: /tf
# Publish: /gazebo/set_link_state

import rospy
import tf
import roslib

from gazebo_msgs.msg import LinkState

def talker():
    rospy.init_node('robot_pose_publisher', anonymous=True)

    robot_pose_pub = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=10)
    robot_pose_msg = LinkState()
    # The name of the link, as in the model-1_4.sdf file.
    robot_pose_msg.link_name = 'taurob'
    # The robot is placed relative to the "map" frame.
    robot_pose_msg.reference_frame = 'map'

    listener = tf.TransformListener()
    # Updating the pose five times a second.
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))

            robot_pose_msg.pose.position.x = trans[0]
            robot_pose_msg.pose.position.y = trans[1]
            robot_pose_msg.pose.orientation.z = rot[2]
            robot_pose_msg.pose.orientation.w = rot[3]

            robot_pose_pub.publish(robot_pose_msg)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass