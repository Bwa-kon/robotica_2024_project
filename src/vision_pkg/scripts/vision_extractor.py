#!/usr/bin/env python3

import math
import cv2 as cv

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_pkg.msg import Object, DetectedObjects

class VisionExtractor:
	def __init__(self):
		# Set up topics publishers and subscribers
		self.image_in_sub = rospy.Subscriber("/vision/image_in", Image, self._image_cb)
		self.image_out_pub = rospy.Publisher("/vision/image_out", Image, queue_size=4)
		self.detections_sub = rospy.Subscriber("/vision/detections", DetectedObjects, self._detections_cb)
		self.cv_bridge = CvBridge()

		# Initialize class properties
		self.detected_objs = []

	def _image_cb(self, data):
		try:
			# Convert image message to an image frame
			self.frame = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			rospy.logwarn(e)
		else:
			# Draw bounding boxes of detected objects
			self._draw_boxes(self.frame)

			try:
				# Convert frame back to a ROS image message
				img_msg = self.cv_bridge.cv2_to_imgmsg(self.frame, "bgr8")
			except CvBridgeError as e:
				rospy.logwarn(e)
			else:
				# Publish the output image
				self.image_out_pub.publish(img_msg)

			# Show the output image
			cv.imshow("Camera preview", self.frame)

		# Shutdown if 'q' is pressed
		if cv.waitKey(1) == ord('q'):
			rospy.signal_shutdown("User requested shutdown")

	def _detections_cb(self, data):
		# Save array of detected objects
		self.detected_objs = data.objects

	def _get_color(self, i):
		color0 = (0, 0, 0)
		colors = [(0, 0, 192),
		          (192, 0, 192),
		          (192, 192, 0),
		          (0, 192, 0),
		          (0, 192, 192),
		          (192, 0, 0),
		          (192, 0, 128),
		          (0, 128, 192)]

		if i < len(colors):
			return colors[i]
		else:
			return color0

	def _draw_boxes(self, frame):

		for obj in self.detected_objs:
			# Get object properties
			label = obj.label
			confidence = obj.confidence
			rad = obj.radians
			xmin = obj.xmin
			ymin = obj.ymin
			xmax = obj.xmax
			ymax = obj.ymax
			width = obj.width
			height = obj.height
			color = self._get_color(obj.id)

			# Draw angle line
			cv.line(frame, (xmin, ymin), (int(xmin + width * math.cos(rad)), int(ymin + height * math.sin(rad))), color, 2)

			# Draw text
			text = "%s: %.2f" % (label, confidence)
			cv.putText(frame, text, (xmin, ymin - 5), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv.LINE_AA)

			# Get bounding box dimensions
			p1 = (xmin, ymin)
			p2 = (xmax, ymax)

			# Draw bounding box
			cv.rectangle(frame, p1, p2, color, 2)

def main():
	rospy.init_node("vision_extractor", anonymous=True)

	vision = VisionExtractor()

	rospy.spin()

if __name__ == '__main__':
	main()
