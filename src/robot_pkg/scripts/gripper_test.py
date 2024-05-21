#!/usr/bin/env python
import rospy
import time

from math import pi

from niryo_robot_python_ros_wrapper import *

from niryo_robot_led_ring.api import LedRingRosWrapper
from niryo_robot_sound.api import SoundRosWrapper
from std_msgs.msg import ColorRGBA


# from niryo_robot_tools_commander.msg import ToolGoal, ToolAction
sound = SoundRosWrapper()
niryo_robot = NiryoRosWrapper()
led_ring= LedRingRosWrapper()
offcet = 0.20

table_pose = [1.46,0.22,-0.05,-0.0,-1.75,0]
test_position = [0.0219560326463, 0.196838727165, 0.408371233569, 3.13203152527, 1.55955233502, -1.69000684762]


def playSound():

    sound.play(sound.sounds[0])
    # sound.play(sound.sounds[1])
    # sound.play(sound.sounds[2])
    # sound.play(sound.sounds[3])
    # sound.play(sound.sounds[4])

def changeColor():
    #led_ring.turn_off()
    led_ring.solid(color=[255, 255, 255])
    #led_ring.solid(ColorRGBA(r=255,g=255,b=255),True)



def move_to_pose(pose):
    niryo_robot.move_joints(pose[0],pose[1],pose[2],pose[3],pose[4],pose[5])

def move_to_pre(pose):
    niryo_robot.move_pose(pose[0],pose[1],pose[2]+offcet,pose[3],pose[4],pose[5],frame="bakje_3")

def move_to_cord(pose):
    niryo_robot.move_pose(pose[0],pose[1],pose[2],pose[3],pose[4],pose[5])

def place(box_nr):
    niryo_robot.move_pose(0.05,-0.08,0.20,-pi/2,pi/2,0,frame="bakje_"+str(box_nr))
    niryo_robot.release_with_tool()





if __name__ =='__main__':
    print("test")

    rospy.init_node('gripper_test',anonymous=True )
    niryo_robot.grasp_with_tool()


    #niryo_robot.calibrate_auto() b
    niryo_robot.calibrate_manual()
    niryo_robot.update_tool()

   # rospy.loginfo(niryo_robot.get_pose())

    sound.set_volume(10)

    #niryo_robot.move_joints(0.1,-0.2,0.0,1.1,-0.5,0.2)
    # move_to_pose(table_pose)
    #move_to_cord(test_position)

    #niryo_robot.move_pose_saved("home")
    place(4)
    place(3)
    place(2)
    place(1)


    while not rospy.is_shutdown() and True:


        sound.play(sound.sounds[1], wait_end = False)
        niryo_robot.release_with_tool()
        time.sleep(1)

        sound.play(sound.sounds[0],wait_end = False)
        niryo_robot.grasp_with_tool()
        time.sleep(1)
        continue



    # action_client = actionlib.SimpleActionClient('/niryo_robot_tools_commander/action_server',ToolAction)

    # action_client.wait_for_server()

    # goal= ToolGoal()
    # goal.cmd = 2

    # action_client.send_goal(goal)

    # action_client.wait_for_result()

