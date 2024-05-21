#!/bin/bash

set -e

# Save current path working directory
DIR="${PWD}"

# Get location of this script
SCRIPT_LOC=`dirname "$0"`
IP=${ROS_CONTROLLER_IP}

cd "${SCRIPT_LOC}/.."

sshpass -p robotics ssh niryo@${IP} "mkdir -p /home/niryo/niryo_robot_ws"
sshpass -p robotics ssh niryo@${IP} "mkdir -p /home/niryo/niryo_robot_ws/src"

echo "Uploading  my_niryo_robot_hardware_interface files to robot..."
sshpass -p robotics scp -r my_niryo_robot_hardware_interface niryo@${IP}:/home/niryo/niryo_robot_ws/src

echo "Uploading  my_niryo_robot_description files to robot..."
sshpass -p robotics scp -r my_niryo_robot_description niryo@${IP}:/home/niryo/niryo_robot_ws/src

echo "Updating moveit config of robot ned..."
sshpass -p robotics scp -r my_niryo_ned_moveit_config niryo@${IP}:/home/niryo/niryo_robot_ws/src

echo "Updating moveit config of robot ned2..."
sshpass -p robotics scp -r my_niryo_ned2_moveit_config niryo@${IP}:/home/niryo/niryo_robot_ws/src

echo "Updating moveit config of robot common..."
sshpass -p robotics scp -r my_niryo_moveit_config niryo@${IP}:/home/niryo/niryo_robot_ws/src

echo "Uploading  my_niryo_robot_bringup files to robot..."
sshpass -p robotics scp -r my_niryo_robot_bringup niryo@${IP}:/home/niryo/niryo_robot_ws/src

echo "Uploading  my_niryo_robot_arm_commander files to robot..."
sshpass -p robotics scp -r my_niryo_robot_arm_commander niryo@${IP}:/home/niryo/niryo_robot_ws/src

echo "Ready"
echo
echo "Do not forget to build workspace and source it on the robot"
echo "Commands on the robot terminal:"
echo "source /home/catkin_ws/install/release/ned/setup.bash<NED>"
echo " --- OR ---"
echo "source /home/catkin_ws/install/release/ned2/setup.bash<NED2>"
echo "$ cd /home/niryo/niryo_robot_ws"
echo "$ catkin_make clean" # not always nessery
echo "$ catkin_make"
echo "$ source devel/setup.bash"

# Change path back to where we were
cd "${DIR}"
