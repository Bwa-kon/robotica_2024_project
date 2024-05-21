#!/user/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header





def talker():
    rospy.init_node('gripper_joint_state_publicher')

    pub = rospy.Publisher('joint_states', JointState ,queue_size=10)
    rate = rospy.Rate(10)

    state = JointState()
    state.header = Header()
    state.name = ['joint_base_to_mors_1','joint_base_to_mors_2']
    state.position =[0.00,-0.01]
    state.velocity = state.position
    state.effort = state.position

    while not rospy.is_shutdown():
        pub.publish(state)
        state.header.stamp = rospy.Time.now()
        rate.sleep()








if __name__ =='__main__':

    talker()