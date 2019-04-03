#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionGoal

def send_navigation_command(pub):
    try:
        goal_x = rospy.get_param('/eva/fixedSensorX') - 1.0
        goal_y = rospy.get_param('/eva/fixedSensorY') + 2.0
    except:
        print("Fixed gas sensor position is unknown.")
        return
       
    msg = MoveBaseActionGoal()
    msg.goal.target_pose.header.frame_id = "map"
    msg.goal.target_pose.pose.position.x = goal_x
    msg.goal.target_pose.pose.position.y = goal_y
    msg.goal.target_pose.pose.orientation.w = 1.0

    rospy.sleep(0.5)
    pub.publish(msg)


def callback(data):
    gas_level = float(data.data)

    if gas_level > 30.0:
        return True

    return False

    
def listener():

    rospy.init_node('gas_sensor_monitoring', anonymous=True)

    pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)

    send_command = rospy.Subscriber('/eva/fixed_gas_level', String, callback)

    rospy.sleep(10)

    if send_command:
        send_navigation_command(pub)

    rospy.spin()

if __name__ == '__main__':
    listener()