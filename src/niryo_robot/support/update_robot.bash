#!/bin/bash

set -e

# Save current path working directory
DIR="${PWD}"

# Get location of this script
SCRIPT_LOC=`dirname "$0"`
IP=${ROS_CONTROLLER_IP}

cd "${SCRIPT_LOC}/.."

echo "Updating urdf of robot..."
sshpass -p robotics scp -r my_niryo_robot_description niryo@${IP}:/home/niryo/niryo_robot_ws/src

echo "Updating moveit config of robot ned..."
sshpass -p robotics scp -r my_niryo_ned_moveit_config niryo@${IP}:/home/niryo/niryo_robot_ws/src

echo "Updating moveit config of robot ned2..."
sshpass -p robotics scp -r my_niryo_ned2_moveit_config niryo@${IP}:/home/niryo/niryo_robot_ws/src

echo "Updating moveit config of robot common..."
sshpass -p robotics scp -r my_niryo_moveit_config niryo@${IP}:/home/niryo/niryo_robot_ws/src

echo "Ready"

# Change path back to where we were
cd "${DIR}"
