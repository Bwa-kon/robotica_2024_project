#!/usr/bin/env python3

import os
import json
import time
import numpy as np
import cv2 as cv

import rospy
import actionlib
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, Quaternion
from cv_bridge import CvBridge, CvBridgeError
from vision_pkg.msg import Object, DetectedObjects
from vision_pkg.msg import GetObjectsAction, GetObjectsGoal, GetObjectsResult, GetObjectsFeedback

import depthai

class CameraPublisher:
	def __init__(self):
		self.image_in_pub = rospy.Publisher("/vision/image_in", Image, queue_size=1)
		self.detections_pub = rospy.Publisher("/vision/detections", DetectedObjects, queue_size=1)
		self.action_srv = actionlib.SimpleActionServer('/vision/get_objects', GetObjectsAction, self._get_objects_goal_cb, auto_start=False)
		self.cv_bridge = CvBridge()

	def startup(self):
		# Create pipeline
		self.pipeline = depthai.Pipeline()

		# Add color camera
		cam_rgb = self.pipeline.createColorCamera()
		cam_rgb.setPreviewSize(416, 416)
		cam_rgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_1080_P)
		cam_rgb.setInterleaved(False)
		cam_rgb.setColorOrder(depthai.ColorCameraProperties.ColorOrder.BGR)
		cam_rgb.setFps(30)

		# Set camera properties
		cam_ctrl = depthai.CameraControl()
		cam_ctrl.setAutoFocusMode(depthai.CameraControl.AutoFocusMode.AUTO)

		# Add mono cameras
		cam_left = self.pipeline.createMonoCamera()
		cam_right = self.pipeline.createMonoCamera()
		cam_left.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_400_P)
		cam_right.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_400_P)
		cam_left.setBoardSocket(depthai.CameraBoardSocket.CAM_B)
		cam_right.setBoardSocket(depthai.CameraBoardSocket.CAM_C)

		# Add depth map
		stereo = self.pipeline.createStereoDepth()
		stereo.setDefaultProfilePreset(depthai.node.StereoDepth.PresetMode.HIGH_DENSITY)
		stereo.setDepthAlign(depthai.CameraBoardSocket.CAM_A)
		stereo.setOutputSize(cam_left.getResolutionWidth(), cam_left.getResolutionHeight())
		stereo.setSubpixel(True)

		# Set up communication from device to host
		xout_rgb = self.pipeline.createXLinkOut()
		xout_nn = self.pipeline.createXLinkOut()
		xout_depth = self.pipeline.createXLinkOut()
		xout_nn_network = self.pipeline.createXLinkOut()
		xout_rgb.setStreamName("rgb")
		xout_nn.setStreamName("detections")
		xout_depth.setStreamName("depth")
		xout_nn_network.setStreamName("nn_network")

		# Define YOLOv5 model to use
		cwd = os.getcwd()
		yolo_blob = "best"
		yolo_blob_file = os.path.abspath(cwd + "/../blobs/" + yolo_blob + ".blob")
		yolo_blob_json = os.path.abspath(cwd + "/../blobs/" + yolo_blob + ".json")

		# Read model json
		yolo_json = open(yolo_blob_json)
		yolo_data = json.load(yolo_json)
		yolo_nn_config = yolo_data["nn_config"]["NN_specific_metadata"]

		# Extract model properties
		yolo_num_classes = yolo_nn_config["classes"]
		yolo_coordinate_size = yolo_nn_config["coordinates"]
		yolo_anchors = yolo_nn_config["anchors"]
		yolo_anchor_masks = yolo_nn_config["anchor_masks"]
		yolo_iou_th = yolo_nn_config["iou_threshold"]
		yolo_confidence_th = yolo_nn_config["confidence_threshold"]

		# Extract object labels
		self.labels = yolo_data["mappings"]["labels"]

		# Define other YOLOv5 model properties
		yolo_bounding_box_scale = 0.5
		yolo_depth_lower = 100 # 10 centimeters !VERY IMPORTANT!
		yolo_depth_upper = 5000 # 5 meters !VERY IMPORTANT!

		# Define YOLOv5 detection network to use
		yolo_spatial = self.pipeline.createYoloSpatialDetectionNetwork()
		yolo_spatial.setBlobPath(yolo_blob_file)

		# Spatial detection specific parameters
		yolo_spatial.setConfidenceThreshold(yolo_confidence_th)
		yolo_spatial.setBoundingBoxScaleFactor(yolo_bounding_box_scale)
		yolo_spatial.setDepthLowerThreshold(yolo_depth_lower)
		yolo_spatial.setDepthUpperThreshold(yolo_depth_upper)
		yolo_spatial.input.setBlocking(False)

		# YOLO specific parameters
		yolo_spatial.setNumClasses(yolo_num_classes)
		yolo_spatial.setCoordinateSize(yolo_coordinate_size)
		yolo_spatial.setAnchors(yolo_anchors)
		yolo_spatial.setAnchorMasks(yolo_anchor_masks)
		yolo_spatial.setIouThreshold(yolo_iou_th)

		# Link nodes together
		cam_rgb.preview.link(yolo_spatial.input)
		cam_left.out.link(stereo.left)
		cam_right.out.link(stereo.right)
		yolo_spatial.passthrough.link(xout_rgb.input)
		yolo_spatial.out.link(xout_nn.input)
		stereo.depth.link(yolo_spatial.inputDepth)
		yolo_spatial.passthroughDepth.link(xout_depth.input)
		yolo_spatial.outNetwork.link(xout_nn_network.input)

		self._get_device()
		self.action_srv.start()
		rospy.loginfo("CameraPublisher is up")

	def _get_device(self):
		# Find and get available device
		self.device = depthai.Device(self.pipeline)

		# Get output queues from device
		rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
		nn = self.device.getOutputQueue(name="detections", maxSize=4, blocking=False)
		depth = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)
		nn_network = self.device.getOutputQueue(name="nn_network", maxSize=4, blocking=False)
		self.output_queues = (rgb, nn, depth, nn_network)

	def _iterate(self):
		# Fetch data from queues
		(q_rgb, q_nn, q_depth, q_nn_network) = self.output_queues
		try:
			in_rgb = q_rgb.get()
			in_nn = q_nn.get()
			in_depth = q_depth.get()
			in_nn_network = q_nn_network.get()
		except Exception as e:
			rospy.logerr(e)
			rospy.signal_shutdown("Could not get frame")
			return

		# Get OpenCV frame
		try:
			frame = in_rgb.getCvFrame()

			# Check if the frame has any content
			if frame is None:
				# Keep publisher alive
				return
			else:
				try:
					# Convert frame to a ROS image message
					img_msg = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
				except Exception as e:
					rospy.logwarn(e)
				else:
					# Now publish frame
					self.image_in_pub.publish(img_msg)
		except Exception as e:
			rospy.logwarn(e)
			return

		# Get detection array
		detections = in_nn.detections
		self.detected_objects = []

		# Only if there are any detections
		if detections:
			rows = frame.shape[0]
			cols = frame.shape[1]

			for detection in detections:
				# Create data structure for detected object
				obj = Object()

				# Get obj properties
				obj.id = detection.label
				try:
					obj.label = self.labels[detection.label]
				except:
					obj.label = "unknown"
				obj.confidence = detection.confidence

				# Get object location
				obj.pose.position.x = detection.spatialCoordinates.x
				obj.pose.position.y = detection.spatialCoordinates.y
				obj.pose.position.z = detection.spatialCoordinates.z

				# Get bounding box dimensions
				obj.xmin = int(detection.xmin * cols)
				obj.ymin = int(detection.ymin * rows)
				obj.xmax = int(detection.xmax * cols)
				obj.ymax = int(detection.ymax * rows)

				# Calculate object dimensions
				obj.width = obj.xmax - obj.xmin
				obj.height = obj.ymax - obj.ymin

				# Crop image to object
				a = 8 # context
				try:
					obj_img = frame[obj.ymin-a:obj.ymin+obj.height+a*2, obj.xmin-a:obj.xmin+obj.width+a*2]
					obj.radians = self._get_angle(obj_img)
					obj.angle = self._rad_to_degrees(obj.radians)
					obj.pose.orientation.z = obj.radians
				except Exception as e:
					rospy.logdebug(e)

				# Add information to new array
				self.detected_objects.append(obj)

		# Publish the new array
		self.detections_pub.publish(self.detected_objects)

		# Keep publisher alive
		return

	def _get_objects_goal_cb(self, goal):
		rospy.loginfo("Received action goal: retrieve detected objects")

		# Return objects as action result
		result = GetObjectsResult()
		result.objects = self.detected_objects
		self.action_srv.set_succeeded(result)

	def _get_angle(self, cropped_img):
		img = cropped_img

		# Convert to monocolor
		img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

		# Binarize image
		img = cv.adaptiveThreshold(img, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY_INV, 31, 3)

		# Apply morphology operation
		img = cv.erode(img, cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5)))
		img = cv.dilate(img, cv.getStructuringElement(cv.MORPH_ELLIPSE, (11, 11)))

		# Extract external contours
		(contours, hierarchy) = cv.findContours(img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

		# Get largest contour
		cnt = self._get_largest_contour(contours)

		# Get a fitting center line
		[vx, vy, x, y] = cv.fitLine(cnt, cv.DIST_L2, 0, 0.01, 0.01)

		# Calculate the angle using the normalized vector
		flipped = (vy < 0)
		x_axis = [1, 0]
		fit_line = [vx, vy]
		radians = np.arccos(np.dot(x_axis, fit_line))

		try:
			# Convert array to item
			radians = radians.item()
		except:
			pass

		if flipped:
			return -1 * radians
		else:
			return radians

	def _get_largest_contour(self, contours):
		cnt_list = list()

		# Get area for each contour
		for cnt in contours:
			area = cv.contourArea(cnt)
			cnt_list.append((cnt, area))

		# Sort by area size
		cnt_list.sort(key=lambda i: i[1], reverse=True)

		# Return largest contour
		return cnt_list[0][0]

	def _rad_to_degrees(self, rad):
		return round(rad * 180 / np.pi, 2)

	def _degrees_to_rad(self, degrees):
		return round(degrees * np.pi / 180, 2)

	def run(self):
		# Run main loop while iteration succeeds and
		# while the ROS node should not shutdown
		while not rospy.is_shutdown():
			self._iterate()

def main():
	rospy.init_node("camera_publisher", anonymous=True)

	cam = CameraPublisher()

	try:
		cam.startup()
	except Exception as e:
		rospy.logerr(e)
		rospy.signal_shutdown("Error occurred")
	else:
		cam.run()

if __name__ == '__main__':
	main()
