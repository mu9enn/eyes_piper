#!/bin/bash

# Get the current working directory, can be dynamically modified later
WORK_DIR=$(pwd)

# Enable the robotic arm
gnome-terminal -- bash -c "
roslaunch piper start_single_piper.launch gripper_val_mutiple:=2;
exec bash
"

sleep 3

# Start the MoveIt control node for the robotic arm
gnome-terminal -- bash -c "
cd ~/桌面/catkin_ros/piper_ros;
source devel/setup.bash;
roslaunch piper_with_gripper_moveit demo.launch use_rviz:=false;
exec bash
"

# Start the camera node
gnome-terminal -- bash -c "
cd ~/桌面/catkin_ros/vision_ros;
source devel/setup.bash;
roslaunch astra_camera dabai.launch;
exec bash
"

sleep 3

# Start the YOLO detection node for the camera
gnome-terminal -- bash -c "
python run/ros_detect_sunx.py
exec bash
"

# Start the main picking node
python run/pickone_sunx.py