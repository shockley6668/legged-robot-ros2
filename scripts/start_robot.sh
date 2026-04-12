#!/bin/bash
source /opt/ros/humble/setup.bash
source /root/legged-robot/install/setup.bash

# Ensure background processes are killed when this script exits
trap "kill $PID1 $PID2" EXIT

# Start bringup in background
ros2 launch robot_bringup bridge.launch.py &
PID1=$!

# Start control in background
ros2 launch robot_control robot_control.launch.py &
PID2=$!

wait $PID1 $PID2
