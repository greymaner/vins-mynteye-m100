#!/bin/bash

source /home/jiaxing/vins-mynteye-m100/wrappers/ros/devel/setup.bash

sleep 1s 
gnome-terminal -x bash -c "roslaunch dji_sdk sdk.launch;exec bash"
sleep 5s
gnome-terminal -x bash -c "roslaunch mynt_eye_ros_wrapper vins_fusion.launch;exec bash"
sleep 5s
gnome-terminal -x bash -c "rosrun dji_sdk_demo demo_flight_control;exec bash"
sleep 3s
gnome-terminal -x bash -c "rosrun vins vins_node /home/jiaxing/vins-mynteye-m100/wrappers/ros/src/VINS-Fusion/config/mynteye/mynteye_stereo_imu_config.yaml;exec bash"
sleep 2s
gnome-terminal -x bash -c "roslaunch vins vins_rviz.launch;exec bash"