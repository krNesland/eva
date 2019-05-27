#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from move_base_msgs.msg import MoveBaseActionGoal

start_move = False

def send_navigation_command(pub):
    try:
        goal_x = rospy.get_param('/eva/fixedSensorX') - 0.5
        goal_y = rospy.get_param('/eva/fixedSensorY') + 0
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
    global start_move
    gas_level = data.data

    if gas_level > 17.7:
        start_move = True

    
def listener():
    global start_move

    rospy.init_node('gas_sensor_monitoring', anonymous=True)

    pub = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)

    # Waiting a little (just for visualization).
    rospy.sleep(7)

    rospy.Subscriber('/eva/fixed_gas_level', Float32, callback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if start_move:
            send_navigation_command(pub)
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    listener()