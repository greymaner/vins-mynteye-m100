#!/bin/bash

source /home/nvidia/Desktop/vins-mynteye-m100/wrappers/ros/devel/setup.bash

sleep 1s 
gnome-terminal -x bash -c "roslaunch dji_sdk sdk.launch;exec bash"
sleep 5s
gnome-terminal -x bash -c "roslaunch mynt_eye_ros_wrapper mynteye.launch;exec bash"
sleep 5s
gnome-terminal -x bash -c "rosrun dji_sdk_demo demo_flight_control;exec bash"
sleep 3s
gnome-terminal -x bash -c "roslaunch vins_estimator mynteye.launch;exec bash"
