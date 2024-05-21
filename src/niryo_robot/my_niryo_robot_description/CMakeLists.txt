cmake_minimum_required(VERSION 2.8.3)
project(my_niryo_robot_description)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    # our packages are listes here
    #my_niryo_robot_description
    std_msgs
    actionlib_msgs
    geometry_msgs
)

## Generate actions in the 'action' folder
add_action_files(
    FILES
    ActionMsg.action
    scan_pose.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    #my_niryo_robot_description
    actionlib_msgs
    std_msgs
    geometry_msgs

    # our package go here
)

catkin_package(
    CATKIN_DEPENDS
    rospy actionlib_msgs
    # your package dependencies go here
)
