#!/usr/bin/env python2

import rospy
import time

from math import pi

from niryo_robot_python_ros_wrapper import *

from niryo_robot_led_ring.api import LedRingRosWrapper
from niryo_robot_sound.api import SoundRosWrapper

#import for actionserver
import actionlib
from my_niryo_robot_description.msg import ActionMsgAction, ActionMsgFeedback, ActionMsgResult
from my_niryo_robot_description.msg import scan_poseAction, scan_poseFeedback, scan_poseResult


# define draaitafel pose for the ned
table_pose=[1.36,0.33,0.35,-0.11,-1.92,-0.05]

# this class is used as a stucture so we can acces the min and max of a variable
class range:
    min = 0
    max = 0


class Pick_and_place:
## go towards scan pose
    def __init__(self,ScanPose):
        rospy.loginfo("Starting pick and place program")
## initialising and bootup process
        # initialize controlller objects for niryo
        # self.sound = SoundRosWrapper()
        # self.led_ring= LedRingRosWrapper()
        self.niryo_robot = NiryoRosWrapper()
        self.niryo_robot.wait_for_nodes_initialization()

        self.niryo_robot.update_tool()
        rospy.loginfo(self.niryo_robot.get_current_tool_id())


        self.niryo_robot.reboot_motors()
        self.niryo_robot.enable_tcp(True)
        self.niryo_robot.request_new_calibration()
        self.niryo_robot.calibrate_auto()
        self.niryo_robot.set_arm_max_velocity(50)
        self.niryo_robot.set_arm_max_acceleration(50)

        # def poses
        self.ScanPose = ScanPose
#look what the offset is for
        self.ofset =0.05

        # get the box size from param server
        self.PlaceOffset = [0.0,-0.08,0.1,pi/2,pi/2,0]

        # self.program()


        # creates the action server
        self.pnp_as = actionlib.SimpleActionServer("robot/pnp_as", ActionMsgAction, self.goal_callback, False)
        self.pnp_as.start()

        self.startpos_as = actionlib.SimpleActionServer("robot/startpos_as", scan_poseAction, self.scanpose_callback, False)
        self.startpos_as.start()

    # function to move the robot to a position using joint coodinates
    def move_joints_pose(self,pose):
        self.niryo_robot.move_joints(pose[0],pose[1],pose[2],pose[3],pose[4],pose[5])

    # funcion to move the robot tcp to a coordinate
    def move_to_cord(self,cord):
        self.niryo_robot.move_pose(cord[0],cord[1],cord[2],
                                   cord[3],cord[4],cord[5],"camera")

    # function to move to a position with a hight offset
    def move_to_pre(self,cord):
        self.niryo_robot.move_pose(cord[0],cord[1],cord[2]+self.ofset,
                                   cord[3],cord[4],cord[5],"camera")

    # function to pick up an item
    def pick(self,position):
        # move the gripper to the pre grasp position
        self.move_to_pre(position)

        # open the gripper
        self.niryo_robot.release_with_tool()

        # move the gripper to the grasp position
        self.niryo_robot.move_pose(self.ofset,0,0, 0,0,0,"TCP")
        time.sleep(1)

        # grasp
        self.niryo_robot.grasp_with_tool()
        time.sleep(1)

        # retreat to avoid collisions
        self.niryo_robot.move_pose(-self.ofset,0,0,0,0,0,"TCP")

    # function to drop the items in the box
    def place(self,box_nr):
        # move to the place position of the box
        if box_nr >4:
            self.PlaceOffset[3] =pi/2
        else:
            self.PlaceOffset[3] =-pi/2

        self.niryo_robot.move_pose(
            self.PlaceOffset[0],self.PlaceOffset[1],self.PlaceOffset[2],    # get the xyz position of the center of the box
            self.PlaceOffset[3],self.PlaceOffset[4],self.PlaceOffset[5],    # get the right rotation for the gripper
            frame="bakje_"+str(box_nr))                                     # move to the right box

        rospy.sleep(1)

        self.niryo_robot.move_pose(0.03,0,0,0,0,0,"TCP")

        # drop the item
        self.niryo_robot.release_with_tool()

        self.niryo_robot.move_pose(-0.03,0,0,0,0,0,"TCP")

    def program(self):
        self.niryo_robot.grasp_with_tool()
        # wait for start signal

        #self.move_joints_pose(self.ScanPose)

        # wait for item cords en type

        # pickup item
        #self.pick([0.0219560326463, 0.196838727165, 0.408371233569, 3.13203152527, 1.55955233502, -1.69000684762])

        # place for right type
        self.niryo_robot.grasp_with_tool()
        self.place(1)
        self.niryo_robot.grasp_with_tool()
        self.place(2)
        self.niryo_robot.grasp_with_tool()
        self.place(3)
        self.niryo_robot.grasp_with_tool()
        self.place(4)
        self.niryo_robot.grasp_with_tool()
        self.place(5)
        self.niryo_robot.grasp_with_tool()
        self.place(6)
        self.niryo_robot.grasp_with_tool()
        self.place(7)
        self.niryo_robot.grasp_with_tool()
        self.place(8)

    ## callback from actionserver
    def goal_callback(self, goal):

        rate = rospy.Rate(1)
        success = True
        Busy = True
        # create messages that are used to publish feedback/result
        feedback = ActionMsgFeedback()
        result = ActionMsgResult()

        # publish info to the console for the user
        # rospy.loginfo('"action_server": Executing, creating action_server sequence of order %i with seeds %i, %i' % ( goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
        # objecttype = goal.id
        # for i in range(1, goal.id):
        # if self._as.is   is_preempt_requestes():
        #     rospy.loginfo("The goal had been cancelled")
        #     success = False
        #     return

        x_pose = goal.object.position.x/1000
        y_pose = goal.object.position.y/1000
        z_pose = goal.object.position.z/1000
        x_orien = goal.object.orientation.x
        y_orien = goal.object.orientation.y -pi/2
        z_orien = goal.object.orientation.z +pi/2

        object_pose = [x_pose,y_pose,z_pose,x_orien,y_orien,z_orien]

        rospy.loginfo("object pose is: %s" ,object_pose)

        self.pick(object_pose)
        self.place(goal.id + 1)


        result.placed =True

        rate.sleep()


        if success:
            self.pnp_as.set_succeeded(result)
            rospy.loginfo('Succeeded sorting object num: %i', goal.id+1)

## Scan objects and move to the alloted bin/container.

    def scanpose_callback(self, goal):

        rate = rospy.Rate(1)
        success = True

        # create messages that are used to publish feedbac
        feedback = scan_poseFeedback()
        result = scan_poseResult()

        # define draaitafel pose for the ned
        #table_pose=[1.46,0.22,-0.05,-0.0,-1.75,0]

        self.move_joints_pose(self.ScanPose)

        result.Ready = True

        rate.sleep()

        if success:
            self.startpos_as.set_succeeded(result)
            rospy.loginfo('Ready to scan for new product')

## 
    def rest_callback(self):
        self.niryo_robot.move_to_sleep_pose()

    def estop_callback(self):
        self.niryo_robot.stop_move()
        self.niryo_robot.pick_and_place




if __name__ =='__main__':
    rospy.init_node('gripper_test',anonymous=True)

    niryo = Pick_and_place(table_pose)

    #Pick_and_place.program()

    rospy.spin()

