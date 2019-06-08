# Eva
Mobile robot for inspection on an unmanned offshore production platform. A proof of concept, developed with the ROS software and provides a web browser interface.

![alt text](https://raw.githubusercontent.com/krNesland/eva/master/img/mode4.gif "Mode 4")

## System Requirements
- Ubuntu 16.04 LTS
- Python 2.7
- ROS Kinetic

Follow the [manual](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#pc-setup) to install the correct version of Linux and ROS. Then, follow the [tutorial](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) to set up a workspace. After this, clone this repository into the "src" folder of your workspace, cd into the workspace and build with 
```
catkin_make
```
and then
```
catkin_make install
```


## Required ROS Packages
- cv_bridge
- opencv3
- tf2_web_republisher
- web_video_server
- image_transport
- rosbridge_server

Packages are installed with 
```
sudo apt-get install ros-kinetic-web-video-server
```
(and similar). Some of the packages listed might be already installed.

## Getting Started
To get started with the TurtleBot, one should go through the steps in the [manual](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview). If not familiar with ROS, one should also go through the [beginner level tutorials](http://wiki.ros.org/ROS/Tutorials).

## How To Run a Mode
Mode 4 as an example. 
```
roslaunch eva_a mode4.launch
```
and open the corresponding web page in the "pages" folder. Note that this mode requires you to have brought up the physical TurtleBot. 
```
roslaunch eva_a mode4_sim.launch
```
can be called for the Gazebo simulation.

## Folder Description
**action, msg and srv** include the definition of the actionlib actions, the messages and the services.

**css, js and pages** include the files necessary css-, javascript- and html files for the web browser interface.

**img** includes image files used in the web browser interface and to store the images of Mode 4.

**map** includes the occupancy grid map used for navigation and Leaflet files.

**models** includes the CAD model of the room and the robot. The robot is only included when not running a simulation. This is because the simulation includes the model from a TurtleBot3 package.

**scripts** include the Python files that run in the nodes.

**worlds** includes the setup of the Gazebo world and controls the import of models.

## Implemented Nodes
- drive_around_server
- follow_route_server
- gas_level_publisher
- gas_sensor_monitoring
- obstacle_publisher
- robot_pose_publisher
- scan_mismatches
- take_picture_server
- thermal_center

## Messages
- Obstacles
- ScanMismatches

## Services
- DriveAround
- TakePicture

## Actions
- FollowRoute
