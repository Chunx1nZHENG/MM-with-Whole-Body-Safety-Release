#!/bin/bash

rosrun xacro xacro -o ../urdf/michelabi.urdf ../urdf/michelabi.xacro \
    base_height:=0.2 \
    tool_name:=none \
    fix_gripper_joints:=false \
    has_realsense_d435i:=false \
    has_realsense_l515:=false \
    has_lidar:=false \
    simulation:=true \
    update_rate:=250 \
    ft_update_rate:=250 \
    fix_sensor:=true \
    has_base_ft_sensor:=false \
    ros_control:=true 