IP=`ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | cut -f1  -d'/'`

export ROS_CONTROLLER_IP=192.168.2.20
#export ROS_DEVELOPMET_IP=192.168.2.11
export ROS_DEVELOPMET_IP=${IP}

export ROS_MASTER=$ROS_CONTROLLER_IP
export ROS_MASTER_URI=http://$ROS_CONTROLLER_IP:11311
export ROS_IP=$ROS_DEVELOPMET_IP
