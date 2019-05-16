#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    listener = tf.TransformListener()

    xVec = []
    yVec = []

    rate = rospy.Rate(2.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        xVec.append(trans[0])
        yVec.append(trans[1])

        print(xVec)
        print(yVec)

        rate.sleep()