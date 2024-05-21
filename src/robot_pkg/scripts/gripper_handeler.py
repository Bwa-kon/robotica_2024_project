#!/user/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tools_interface.msg import Tool


# this class is used a a stucture so we can acces the min and max of a variable
class servoRange:
    min = 0
    max = 0

# this class recives the servostate of the niryo robot and calculates the state of the gripper fingers
class Gripper_state_handler:

    # this function initializes the class
    def __init__(self,finger_names,real_stroke_length,servo_range):
        #set some variables for the class
        self.stroke_length = real_stroke_length
        self.servo_range = servo_range

        # set up the listener
        self.sub = rospy.Subscriber('/niryo_robot_hardware/tools/motor',Tool,self.publish_state)

        # set up the state publicher
        self.pub = rospy.Publisher('/joint_states',JointState,queue_size=10)

        # prepare a msg for the publisher
        self.state = JointState()
        self.state.header = Header()
        self.state.name = finger_names
        self.state.position =[0.0,0.0]
        self.state.velocity = [0.0,0.0]
        self.state.effort = [0,0]

        rospy.loginfo("the gripper state handler is online")

    # in this function the actual state of the state of the fingers of the girpper is calculated in published
    def publish_state(self,msg):
        # calculate the factor of how mutch the fingers shud move
        move_factor =  1.00-(1.00/(self.servo_range.max - self.servo_range.min)*(msg.position-self.servo_range.min))

        # set the right posintion in the state msg
        #self.state.position = [self.stroke_length[0] * move_factor -0.01,self.stroke_length[1] * move_factor-0.01]
        self.state.position =[0,0]

        # set the right time in the header of the msg
        self.state.header.stamp = rospy.Time.now()

        # publish the data so the actual servo state is visible in the virtual envirement
        self.pub.publish(self.state)


# this is the main function of the programm
if __name__ =='__main__':
    # a node is initialized
    rospy.init_node('gripper_joint_state_publicher')

    # a variable of the names of the gripper
    finger_names =['joint_base_to_mors_1','joint_base_to_mors_2']

    # a variable thath contains data over how mutch eatch finger of the gripper can move
    stroke_lengths = [0.02,0.02]

    # a variable with the minimum and maximum servo state
    servo_range = servoRange()
    servo_range.max = 2022
    servo_range.min = 848

    # initialize the class for handeling the gripper state
    gripper_handele = Gripper_state_handler(finger_names,stroke_lengths,servo_range)

    # keep the code active so the handeler can work
    rospy.spin()