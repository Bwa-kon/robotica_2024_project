#!/usr/bin/env python
import rospy

from std_msgs.msg import Bool
#import for action client pick and place
import actionlib
from my_niryo_robot_description.msg import ActionMsgAction, ActionMsgGoal
from geometry_msgs.msg import Pose
from my_niryo_robot_description.msg import scan_poseAction, scan_poseGoal


class HMI_class:
# define HMI subscribers and declare variables
	ros_subs = {"/HMI/StartStop":		{"data": False,
											 "type": Bool},
				"/HMI/Reset":			{"data": False,
											 "type":  Bool},
				"HMI/Estop":			{"data": False,
											"type": Bool},
				"HMI/SingleCycle":		{"data": False,
											"type": Bool}}

# define HMI publishers and declare variables
	ros_pubs = {"HMI/Ready":			{"data": False,
										"type": Bool,
										"queue": 1,
										"pub": None},
				"HMI/ActuatorActive":	{"data": False,
										"type": Bool,
										"queue": 1,
										"pub": None},
				"HMI/Placed":			{"data": False,
										"type": type,
										"queue": 1,
										"pub": None},
				"HMI/Error":			{"data": False,
										"type": Bool,
										"queue": 1,
										"pub": None},
				"HMI/Fault":			{"data": False,
										"type": Bool,
										"queue": 1,
										"pub": None}}

	# declare constructor
	def __init__(self):
		self.sub()
		self.pub()


		rospy.loginfo("communication with the HMI is online")

		# self.Ready 			= self.init_Bool_False()
		# self.ActuatorActive = self.init_Bool_False()
		# # self.Placed =
		# self.Error = self.init_Bool_False()
		# self.Fault = self.init_Bool_False()


	# initalize subscribers
	def sub(self):
		for key in self.ros_subs.keys():
			rospy.Subscriber(name=key,
							data_class=self.ros_subs[key]["type"],
							callback=self.callback,
							callback_args=key)


	# callback function to check new status of sub
	def callback(self, msg, key):
		self.ros_subs[key]["data"] = msg.data

		rospy.loginfo("topick %s printed %s" , key, msg.data)

	# initalize publishers
	def pub(self):
		for key in self.ros_pubs.keys():
			self.ros_pubs[key]["pub"] = rospy.Publisher(name = key,
														data_class = self.ros_pubs[key]["type"],
														queue_size = self.ros_pubs[key]["queue"])

			rospy.loginfo("initialized publisher for %s topick",key)


	def init_Bool_False():
		var = Bool()
		var.data = False
		return var

	def get(self,Topick):
		return self.ros_subs[Topick]["data"]

	# def set(self,Topick,Data):
	# 	self






class Robot_class:
	ros_subs = {"Robot/Bussy":		{"data": False,
								 	 "type": Bool}}

	ros_pubs = {"Robot/Estop":		{"data": False,
									 "type": Bool,
									 "queue_size": 1,
									 "pub": None},
				"Robot/Resting":	{"data": False,
									 "type": Bool,
									 "queue_size": 1,
									 "pub": None},
				"Robot/Calibrate":	{"data": False,
									 "type": Bool,
									 "queue_size": 1,
									 "pub": None}}

	#
	def __init__(self):
		self.sub()
		self.pub()

		rospy.loginfo("waiting for pnp_as")
		# create the connection to the action server
		self.pnp_client = actionlib.SimpleActionClient('robot/pnp_as', ActionMsgAction)
		# waits until the action server is up and running
		self.pnp_client.wait_for_server()

		# create the connection to the action server
		self.scanpose_client = actionlib.SimpleActionClient('robot/startpos_as', scan_poseAction)
		# waits until the action server is up and running
		self.scanpose_client.wait_for_server()


		rospy.loginfo("communication with the robot is online")


	def sub(self):
		for key in self.ros_subs.keys():
			rospy.Subscriber(name=key,
							 data_class=self.ros_subs[key]["type"],
							 callback=self.bool_callback,
							 callback_args=key)
			rospy.loginfo("initialized subscriber for %s topick",key)

	def pub(self):
		for key in self.ros_pubs.keys():
			self.ros_pubs[key]["pub"] = rospy.Publisher(key,self.ros_pubs[key]["type"],queue_size= self.ros_pubs[key]["queue_size"])

			rospy.loginfo("initialized publisher for %s topick",key)




	def bool_callback(self,msg,key):
		self.ros_subs[key]["data"] = msg.data

		rospy.loginfo("topick %s printed %s" , key, msg.data)

	def scan_pos(self):
		# creates a goal to send to the action server
		goal = scan_poseGoal()

		# sends the goal to the action server
		self.scanpose_client.send_goal(goal)

		self.scanpose_client.wait_for_result()

		outcome = self.scanpose_client.get_result()

		return outcome

	def pick_and_place(self, coords, ob_type):

		# creates a goal to send to the action server
		goal = ActionMsgGoal(coords, ob_type)
		#goal.id = ob_type
		#goal.object = coords
		# sends the goal to the action server
		self.pnp_client.send_goal(goal)

		self.pnp_client.wait_for_result()

		result = self.pnp_client.get_result()

		return result
		#Test = False

	def calibrate(self):
		Test = False



class Tunrningtable_class:

	test =False

class Vision_class:
	test= False


if __name__ =='__main__':
	rospy.init_node("Main_Ros_program")

	Robot = Robot_class()
	#HMI= HMI_class()



	while (not rospy.is_shutdown()):
		# check if everything is ok and go into start position



		# move the robot to the scan position
		Robot.scan_pos()

		# start looking for objects


		# pick and place
		pose = Pose()
		pose.position.z = 0.05
		result = Robot.pick_and_place(pose,0)
		result = Robot.pick_and_place(pose,1)
		result = Robot.pick_and_place(pose,2)
		result = Robot.pick_and_place(pose,3)
		result = Robot.pick_and_place(pose,4)
		result = Robot.pick_and_place(pose,5)
		result = Robot.pick_and_place(pose,6)
		result = Robot.pick_and_place(pose,7)



		# send results to hmi
		continue





