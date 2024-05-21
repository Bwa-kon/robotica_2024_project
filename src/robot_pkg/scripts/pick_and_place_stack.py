#!/usr/bin/env python3

import rospy
import time

from math import pi

from niryo_robot_python_ros_wrapper import *

from niryo_robot_led_ring.api import LedRingRosWrapper
from niryo_robot_sound.api import SoundRosWrapper

# define poses for the ned
table_pose=[1.46,0.22,-0.05,-0.0,-1.75,0]

# this class is used a a stucture so we can acces the min and max of a variable
class range:
    min = 0
    max = 0


class Pick_and_place:
    def __init__(self,ScanPose):
        rospy.loginfo("Starting pick and place program")

        # initialize controlller objects for niryo
        # self.sound = SoundRosWrapper()
        # self.led_ring= LedRingRosWrapper()
        self.niryo_robot = NiryoRosWrapper()


        self.niryo_robot.update_tool()
        rospy.loginfo(self.niryo_robot.get_current_tool_id())

        self.niryo_robot.enable_tcp(True)


        # def poses
        self.ScanPose = ScanPose

        self.ofset =0.05

        # get the box size from param server
        self.boxSize = [0.05,-0.15,0.08]

        self.program()

    # function to move the robot to a position using joint coodinates
    def move_joints_pose(self,pose):
        self.niryo_robot.move_joints(pose[0],pose[1],pose[2],pose[3],pose[4],pose[5])

    # funcion to move the robot tcp to a cordinate
    def move_to_cord(self,cord):
        self.niryo_robot.move_pose(cord[0],cord[1],cord[2],
                                   cord[3],cord[4],cord[5])

    # function to move to a position with a hight offset
    def move_to_pre(self,cord):
        self.niryo_robot.move_pose(cord[0],cord[1],cord[2]+self.ofset,
                                   cord[3],cord[4],cord[5])

    # function to pick up an item
    def pick(self,position):
        # move the gripper to the pre grasp position
        self.move_to_pre(position)

        # open the gripper
        self.niryo_robot.release_with_tool()

        # move the gripper to the grasp position
        self.move_to_cord(position)
        time.sleep(1)

        # grasp
        self.niryo_robot.grasp_with_tool()
        time.sleep(1)

        # retreed to avoid collisions
        self.move_to_pre(position)

    # function to drop the items in the box
    def place(self,box_nr):
        # move to the place position of the box
        self.niryo_robot.move_pose(
            self.boxSize[0],self.boxSize[1],self.boxSize[2],    # get the xyz position of the center of the box
            0,pi,-pi/2,                                       # get the right rotation for the gripper
            frame="bakje_"+str(box_nr))                         # move to the right box

        self.niryo_robot.move_pose(0.06,0,0,0,0,0,"TCP")

        # drop the item
        self.niryo_robot.release_with_tool()

        self.niryo_robot.move_pose(-0.05,0,0,0,0,0,"TCP")

    def program(self):
        self.niryo_robot.grasp_with_tool()
        # wait for start signal

        # self.move_joints_pose(self.ScanPose)

        # wait for item cords en type

        # pickup item
        #self.pick([0.0219560326463, 0.196838727165, 0.408371233569, 3.13203152527, 1.55955233502, -1.69000684762])

        # place for right type
        self.niryo_robot.grasp_with_tool()
        self.place(6)
        self.niryo_robot.grasp_with_tool()
        self.place(2)
        self.niryo_robot.grasp_with_tool()
        self.place(5)
        self.niryo_robot.grasp_with_tool()
        self.place(1)


if __name__ =='__main__':
    rospy.init_node('gripper_test',anonymous=True)

    niryo = Pick_and_place(table_pose)

    #Pick_and_place.program()

    rospy.spin()

