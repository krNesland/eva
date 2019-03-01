#!/bin/sh


sleep 5
gnome-terminal -e roslaunch turtlebot3_gazebo turtlebot3_eva_a.launch
sleep 5
gnome-terminal -e roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch