#!/user/bin/env python

import rospy
import time
import tf

from tf_conversions import transformations

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped

closed_state = 848
open_state = 2018

gripper_stroke_length= 0.01




class GripperTfPublicher():

    def __init__(self, finger_list,parent, loop_rate=5):

        self.parent_link = parent
        self.finger_list = finger_list
        self.rate = rospy.Rate(loop_rate)

        self.currend_states =[0.00,0.00]


        #tf
        self.broad_caster_tf = tf.TransformBroadcaster()
        self.tf_msg =TransformStamped()
        self.tf_msg.header.frame_id = self.parent_link
        self.tf_msg.transform.translation.x = 0
        self.tf_msg.transform.translation.y = 0
        self.tf_msg.transform.translation.z = 0

        q = transformations.quaternion_from_euler(0,0,0)
        self.tf_msg.transform.rotation.x =q[1]
        self.tf_msg.transform.rotation.y =q[2]
        self.tf_msg.transform.rotation.z =q[3]
        self.tf_msg.transform.rotation.w =q[4]

        #state
        self.state_pub = rospy.Publisher('joint_states', JointState ,queue_size=10)
        self.state = JointState()
        self.state.header = Header()
        self.state.name = self.finger_list
        self.state.position =self.currend_states
        self.state.velocity = self.state.position
        self.state.effort = self.state.position


        time.sleep(1)


    def send_state(self):


        self.state.header.stamp = rospy.Time.now()

        self.state_pub.publish(self.state)

    def send_tf(self):
        for finger in self.finger_list:
            self.tf_msg.header.stamp = rospy.Time.now()

            self.tf_msg.child_frame_id = finger
            self.tf_msg.transform.translation.x = self.state






    def send_data(self,msg):
        currend_state = gripper_stroke_length/(open_state-closed_state)*(msg.position - closed_state)
        self.currend_states = [currend_state , currend_state*-1]

        self.send_state()
        self.send_tf()






if __name__ =='__main__':
    rospy.init_node('gripper_tf_publicher',anonymous=True )
    gripper_list = ["joint_base_to_mors_1","joint_base_to_mors_2"]
    parrend = "base_gripper_1"


    gripper_controler = GripperTfPublicher()

    rospy.Subscriber('/niryo_robot_hardware/tools/motor','tools_interface/Tool',gripper_controler.send_data)

    while not rospy.is_shutdown():
         pass

