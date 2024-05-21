#!/usr/bin/env python

import rospy

import time
import numpy as np

from std_msgs.msg import Bool, Float32, String
import actionlib
import vision_pkg.msg

from my_niryo_robot_description.msg import ActionMsgAction, ActionMsgGoal
from my_niryo_robot_description.msg import scan_poseAction, scan_poseGoal
from geometry_msgs.msg import Pose

# make class for HMI
class HMI_class:
	# define HMI subscribers and declare variables
	ros_subs = { "/HMI/StartStop":     { "data": None,
	                                     "type": Bool },
	             "/HMI/Reset":         { "data": None,
	                                     "type": Bool },
	             "/HMI/Estop":         { "data": None,
	                                     "type": Bool },
	             "/HMI/SingleCycle":   { "data": None,
	                                     "type": Bool } }

	# define HMI publishers and declare variables
	ros_pubs = { "/HMI/Ready":          { "data": None,
	                                      "type": Bool,
	                                      "queue_size": 1,
	                                      "pub": None },
	             "/HMI/ActuatorActive": { "data": None,
	                                      "type": Bool,
	                                      "queue_size": 1,
	                                      "pub": None },
	             "/HMI/Placed":         { "data": None,
	                                      "type": String,
	                                      "queue_size": 1,
	                                      "pub": None },
	             "/HMI/Error":          { "data": None,
	                                      "type": String,
	                                      "queue_size": 1,
	                                      "pub": None },
	             "/HMI/Faulth":         { "data": None,
	                                      "type": String,
	                                      "queue_size": 1,
	                                      "pub": None } }

	# define constructor
	def __init__(self):
		# call function
		self.sub()
		self.pub()

		# publish initial message
		self.set_ready(False)

		rospy.loginfo("communication with the HMI is online")

	# initialize subscribers
	def sub(self):
		for key in self.ros_subs.keys():
			rospy.Subscriber(name=key,
			                 data_class=self.ros_subs[key]["type"],
			                 callback=self.callback,
			                 callback_args=key)

			rospy.loginfo("initialized subscriber for %s topic", key)

	# initialize publishers
	def pub(self):
		for key in self.ros_pubs.keys():
			self.ros_pubs[key]["pub"] = rospy.Publisher(name = key,
			                                            data_class = self.ros_pubs[key]["type"],
			                                            queue_size= self.ros_pubs[key]["queue_size"])

			rospy.loginfo("initialized publisher for %s topic", key)

	# callback function to check new status of sub
	def callback(self, msg, key):
		self.ros_subs[key]["data"] = msg.data

		rospy.logdebug("topic %s printed %s", key, msg.data)

	def get_start_stop(self):
		try:
			return self.ros_subs["/HMI/StartStop"]["data"]
		except:
			return False

	def get_reset(self):
		try:
			return self.ros_subs["/HMI/Reset"]["data"]
		except:
			return False

	def get_e_stop(self):
		try:
			return self.ros_subs["/HMI/Estop"]["data"]
		except:
			return False

	def get_single_cycle(self):
		try:
			return self.ros_subs["/HMI/SingleCycle"]["data"]
		except:
			return False

	def set_ready(self, active):
		data = Bool()
		data.data = active

		self.ros_pubs["/HMI/Ready"]["pub"].publish(data)

	def set_actuator_active(self, active):
		data = Bool()
		data.data = active

		self.ros_pubs["/HMI/ActuatorActive"]["pub"].publish(data)

	def set_placed(self, string):
		if string:
			data = String()
			data.data = string

			self.ros_pubs["/HMI/Placed"]["pub"].publish(data)

	def set_error(self, string):
		if string:
			data = String()
			data.data = string

			self.ros_pubs["/HMI/Error"]["pub"].publish(data)

	def set_fault(self, string):
		if string:
			data = String()
			data.data = string

			self.ros_pubs["/HMI/Faulth"]["pub"].publish(data)

# make class for Robot
class Robot_class:

	# define Robot subscribers and declare variables
	ros_subs = { "/Robot/Busy":      { "data": None,
	                                   "type": Bool } }

	# define Robot publishers and declare variables
	ros_pubs = { "/Robot/Estop":     { "data": None,
	                                   "type": Bool,
	                                   "queue_size": 1,
	                                   "pub": None },
	             "/Robot/Resting":   { "data": None,
	                                   "type": Bool,
	                                   "queue_size": 1,
	                                   "pub": None },
	             "/Robot/Calibrate": { "data": None,
	                                   "type": Bool,
	                                   "queue_size": 1,
	                                   "pub": None } }

	# define constructor
	def __init__(self):
		# calls function
		self.sub()
		self.pub()

		# create the connection to the action server
		self.pnp_client = actionlib.SimpleActionClient('robot/pnp_as', ActionMsgAction)

		# waits until the action server is up and running
		self.pnp_client.wait_for_server()

		# create the connection to the action server
		self.scanpose_client = actionlib.SimpleActionClient('robot/startpos_as', scan_poseAction)

		# waits until the action server is up and running
		self.scanpose_client.wait_for_server()

		rospy.loginfo("communication with the robot is online")

	# initialize subscribers
	def sub(self):
		for key in self.ros_subs.keys():
			rospy.Subscriber(name=key,
			                 data_class=self.ros_subs[key]["type"],
			                 callback=self.callback,
			                 callback_args=key)

			rospy.loginfo("initialized subscriber for %s topic", key)

	# initialize publishers
	def pub(self):
		for key in self.ros_pubs.keys():
			self.ros_pubs[key]["pub"] = rospy.Publisher(key,
			                                            self.ros_pubs[key]["type"],
			                                            queue_size=self.ros_pubs[key]["queue_size"])

			rospy.loginfo("initialized publisher for %s topic", key)

	# callback function to check new status of sub
	def callback(self, msg, key):
		self.ros_subs[key]["data"] = msg.data

		rospy.logdebug("topic %s printed %s", key, msg.data)

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

		# sends the goal to the action server
		self.pnp_client.send_goal(goal)
		self.pnp_client.wait_for_result()

		result = self.pnp_client.get_result()

		return result

	def calibrate(self):
		pass

class Draaitafel_class:
	online = False

	# define Draaitafel subscribers and declare variables
	ros_subs = { "/motor_info/error":         { "data": None,
	                                            "type": Float32 },
	             "/motor_info/current_speed": { "data": None,
	                                            "type": Float32 } }

	# define Draaitafel publishers and declare variables
	ros_pubs = { "/motor_info/setpoint":  { "data": None,
	                                        "type": Float32,
	                                        "queue_size": 1,
	                                        "pub": None },
	             "/motor_info/direction": { "data": None,
	                                        "type": Bool,
	                                        "queue_size": 1,
	                                        "pub": None } }

	# define constructor
	def __init__(self):
		# calls function
		self.sub()
		self.pub()

		# wait for messages to arrive
		rate = rospy.Rate(10)
		while not self.online and not rospy.is_shutdown():
			rate.sleep()

		rospy.loginfo("communication with the Draaitafel is online")

	# initialize subscribers
	def sub(self):
		for key in self.ros_subs.keys():
			rospy.Subscriber(name=key,
			                 data_class=self.ros_subs[key]["type"],
			                 callback=self.callback,
			                 callback_args=key)

			rospy.loginfo("initialized subscriber for %s topic", key)

	# initialize publishers
	def pub(self):
		for key in self.ros_pubs.keys():
			self.ros_pubs[key]["pub"] = rospy.Publisher(name = key,
			                                            data_class = self.ros_pubs[key]["type"],
			                                            queue_size= self.ros_pubs[key]["queue_size"])

			rospy.loginfo("initialized publisher for %s topic", key)

	# callback function to check new status of sub
	def callback(self, msg, key):
		self.online = True
		self.ros_subs[key]["data"] = msg.data

		rospy.logdebug("topic %s printed %s", key, msg.data)

	def _set_speed(self, speed):
		data = Float32()
		data.data = speed

		self.ros_pubs["/motor_info/setpoint"]["pub"].publish(data)

	def on(self):
		self._set_speed(60)

		# Set the desired timer duration in seconds
		time_duration = 0.5
		# Start the timer
		start_time = time.sleep(time_duration)

		self._set_speed(30)

	def off(self):
		data = Float32()
		data.data = 0

		self.ros_pubs["/motor_info/setpoint"]["pub"].publish(data)

class Vision_class:
	online = False

	# define Draaitafel subscribers and declare variables
	ros_subs = { "/vision/detections": { "data": None,
	                                     "type": vision_pkg.msg.DetectedObjects } }

	# define constructor
	def __init__(self):
		# calls function
		self.sub()

		# wait for messages to arrive
		rate = rospy.Rate(10)
		while not self.online and not rospy.is_shutdown():
			rate.sleep()

		rospy.loginfo("communication with the Vision is online")

	# initialize subscribers
	def sub(self):
		for key in self.ros_subs.keys():
			rospy.Subscriber(name=key,
			                 data_class=self.ros_subs[key]["type"],
			                 callback=self.callback,
			                 callback_args=key)

			rospy.loginfo("initialized subscriber for %s topic", key)

	# callback function to check new status of sub
	def callback(self, msg, key):
		self.online = True
		self.ros_subs[key]["data"] = msg

		rospy.logdebug("topic %s printed %s", key, type(msg))

	def get_objects(self):
		objs = self.ros_subs["/vision/detections"]["data"]
		obj_list = objs.objects
		filtered_objs = []

		if len(obj_list):
			for l in obj_list:
				if l.xmin > 42 and l.ymin > 42 and l.xmax < 374 and l.ymax < 374:
					filtered_objs.append(l)

		return filtered_objs

def check_reset(HMI):
	if HMI.get_reset():
		# reset increments
		pass

def main():
	rospy.init_node("robot_main")

	Robot = Robot_class()
	HMI= HMI_class()
	Draaitafel = Draaitafel_class()
	Vision = Vision_class()

	sorted_objs = [ "obj1", "obj2", "obj3", "obj4", "obj5", "obj6", "obj7", "obj8" ]
	rate = rospy.Rate(10)
	start = False
	error = ""
	fault = ""
	obj = 0

	rospy.loginfo("Ready")

	while not rospy.is_shutdown():
		if not HMI.get_start_stop():
			HMI.set_ready(True)
			check_reset(HMI)
		else:
			while not rospy.is_shutdown():
				objects = []

				check_reset(HMI)

				# check if everything is OK and go into start position

				rospy.loginfo("Starting cycle")

				if HMI.get_e_stop():
					while HMI.get_e_stop() and not rospy.is_shutdown():
						rate.sleep()

				# move robot to scan position
				result = Robot.scan_pos()

				if not result:
					rospy.logwarn("Failed to move to scan position")
					continue

				while True:
					# set motor in motion
					Draaitafel.on()

					# start looking for objects
					while len(objects) == 0:
						objects = Vision.get_objects()

					# stop motor
					Draaitafel.off()
					time.sleep(0.5)

					objects = Vision.get_objects()

					if len(objects) > 0:
						obj = objects[0]

						# Set a fixed pickup height
						obj.pose.position.z = 200
						obj.pose.position.x = -1 * obj.pose.position.x
						break

				# pick and place
				result = Robot.pick_and_place(obj.pose, obj.id)

				if result:
					# send results to HMI
					HMI.set_placed(sorted_objs[obj.id])
				else:
					HMI.set_error(error)
					HMI.set_fault(fault)

				rospy.loginfo("Cycle finished")

				# check if stop was pressed
				if not HMI.get_start_stop():
					break

				# check for single cycle
				if HMI.get_single_cycle():
					HMI.set_ready(False)
					while HMI.get_start_stop() and not rospy.is_shutdown():
						rate.sleep()

		rate.sleep()

if __name__ =='__main__':
	main()
