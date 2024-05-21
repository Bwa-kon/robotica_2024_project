#!/bin/bash

echo "Installing Environment"
cd $HOME/niryo_robot_ws/src

sshpass -p robotics ssh niryo@$ROS_CONTROLLER_IP "mkdir -p /home/niryo/niryo_ned_support/environment"
sshpass -p robotics scp -r my_niryo_robot/environment niryo@$ROS_CONTROLLER_IP:/home/niryo/niryo_ned_support

#sshpass -p robotics ssh niryo@$ROS_CONTROLLER_IP "echo "source /home/niryo/catkin_ws/devel/setup.bash" >> ~/.bashrc"
sshpass -p robotics ssh niryo@$ROS_CONTROLLER_IP "echo "source /home/niryo/niryo_ned_support/environment/environment.bash" >> ~/.bashrc"
echo "Ready"

#echo "Installing python3-catkin-tools"
#sshpass -p robotics ssh niryo@$ROS_MASTER "echo robotics | sudo -S apt update -y"
#sshpass -p robotics ssh niryo@$ROS_MASTER "sudo apt-get install python3-catkin-tools"
#echo "Ready"
