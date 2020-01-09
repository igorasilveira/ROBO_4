# ROBO_4

**By:** Igor Silveira up201505172@fe.up.pt

Course: Robotics in MIEIC

Faculty of Engineering of Porto


**OS:** Ubuntu 16.04 LTS

**ROS Distro:** Kinetic

**Build Command:** catkin build -G"Eclipse CDT4 - Unix Makefiles"

**Launch Files RUN FIRST**
> roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch

**Mapping Script**
> rosrun stdr_samples robo_mapping robot0 laser_0

**Must have STDR simulator and gmapping installed **
> sudo apt-get install ros-$ROS_DISTRO-stdr-simulator
> sudo apt install ros-$ROS_DISTRO-gmapping

**Main Script Dependencies:**

gmapping 

catkin

ros_environment

rospack

roslib

cpp_common

rostime

roscpp_traits

roscpp_serialization

genmsg

genpy

message_runtime

gencpp

geneus

gennodejs

genlisp

message_generation

rosbuild

rosconsole

std_msgs

rosgraph_msgs

xmlrpcpp

roscpp

geometry_msgs

message_filters

rosgraph

rosclean

rosmaster

rosout

rosparam

rosunit

roslaunch

roslz4

rosbag_storage

rospy

std_srvs

topic_tools

rosbag

rostopic

rosnode

rosmsg

rosservice

roswtf

sensor_msgs

actionlib_msgs

rostest

actionlib

tf2_msgs

tf2

tf2_py

tf2_ros

tf

nav_msgs

stdr_msgs
