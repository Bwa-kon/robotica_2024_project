#!/bin/bash


if [ -f ${HOME}/catkin_ws/install/release/ned2/setup.bash ]; then
    NIRYO="Ned2"
    echo "NiryoNed2 detected"
elif [ -f ${HOME}/catkin_ws/install/release/ned/setup.bash ]; then
    NIRYO="Ned"
    echo "NiryoNed(version 1) detected"
else
    echo "No setup.bash file found for this version of Niryo"
fi



PS3='Please enter your operating mode: '
options=("With fysical Niryo Ned Robot(Hotspot mode)" "With fysical Niryo Ned Robot(Wifi mode)" "With fysical Niryo Ned Robot(Ethernet mode)" "None")
select opt in "${options[@]}"
do
    case $opt in
        "With fysical Niryo Ned Robot(Hotspot mode)")
            echo "you chose With fysical Niryo Ned Robot(Hotspot mode)"
            if [ "$NIRYO" = 'Ned' ]; then
                cat $HOME/niryo_ned_support/environment/avans_banner_ned.txt
                source $HOME/catkin_ws/install/release/ned/setup.bash
                source $HOME/niryo_ned_support/environment/hotspot_network.bash
                source $HOME/niryo_robot_ws/devel/setup.bash
                sudo service niryo_robot_ros stop
                roslaunch my_niryo_robot_bringup niryo_ned_robot.launch
            elif [ "$NIRYO" = 'Ned2' ]; then
                cat $HOME/niryo_ned_support/environment/avans_banner_ned2.txt
                source $HOME/catkin_ws/install/release/ned2/setup.bash
                source $HOME/niryo_ned_support/environment/hotspot_network.bash
                source $HOME/niryo_robot_ws/devel/setup.bash
                sudo service niryo_robot_ros stop
                roslaunch my_niryo_robot_bringup niryo_ned2_robot.launch
            else
                echo "Invalid robot type"
                exit 1
            fi
	    break
            ;;
        "With fysical Niryo Ned Robot(Wifi mode)")
            echo "you chose With fysical Niryo Ned Robot(Wifi mode)"
            if [ "$NIRYO" = 'Ned' ]; then
                cat $HOME/niryo_ned_support/environment/avans_banner_ned.txt
                source $HOME/catkin_ws/install/release/ned/setup.bash
                source $HOME/niryo_ned_support/environment/wifi_network.bash
                source $HOME/niryo_robot_ws/devel/setup.bash
                sudo service niryo_robot_ros stop
                roslaunch my_niryo_robot_bringup niryo_ned_robot.launch
            elif [ "$NIRYO" = 'Ned2' ]; then
                cat $HOME/niryo_ned_support/environment/avans_banner_ned2.txt
                source $HOME/catkin_ws/install/release/ned2/setup.bash
                source $HOME/niryo_ned_support/environment/wifi_network.bash
                source $HOME/niryo_robot_ws/devel/setup.bash
                sudo service niryo_robot_ros stop
                roslaunch my_niryo_robot_bringup niryo_ned2_robot.launch
            else
                echo "Invalid robot type"
                exit 1
            fi
            break
            ;;
        "With fysical Niryo Ned Robot(Ethernet mode)")
            echo "you chose With fysical Niryo Ned Robot(Ethernet mode)"
            if [ "$NIRYO" = 'Ned' ]; then
                cat $HOME/niryo_ned_support/environment/avans_banner_ned.txt
                source $HOME/catkin_ws/install/release/ned/setup.bash
                source $HOME/niryo_ned_support/environment/ethernet_network.bash
                source $HOME/niryo_robot_ws/devel/setup.bash
                sudo service niryo_robot_ros stop
                roslaunch my_niryo_robot_bringup niryo_ned_robot.launch
            elif [ "$NIRYO" = 'Ned2' ]; then
                cat $HOME/niryo_ned_support/environment/avans_banner_ned2.txt
                source $HOME/catkin_ws/install/release/ned2/setup.bash
                source $HOME/niryo_ned_support/environment/ethernet_network.bash
                source $HOME/niryo_robot_ws/devel/setup.bash
                sudo service niryo_robot_ros stop
                roslaunch my_niryo_robot_bringup niryo_ned2_robot.launch
            else
                echo "Invalid robot type"
                exit 1
            fi
            break
            ;;
        "None")
            echo "you chose Without fysical Niryo Ned Robot"
            source $HOME/catkin_ws/devel/setup.bash
	    break
            ;;
        *) echo "invalid option $REPLY";;
    esac
done
