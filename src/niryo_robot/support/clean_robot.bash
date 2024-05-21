#!/bin/bash

IP=${ROS_CONTROLLER_IP}

echo "Removing my_niryo_robot files on robot."
sshpass -p robotics ssh niryo@${IP} "rm -rf /home/niryo/catkin_ws/src/my_niryo_robot"
echo "Ready"

