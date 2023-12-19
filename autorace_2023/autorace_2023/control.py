import os
import numpy as np

from cv_bridge import CvBridge
import cv2

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

class Controller(Node):
	def __init__(self, cwd):
		super().__init__('controller')
		self.mode_subscriber = self.create_subscription(
			String,
			'/detect/signs/mode',
			self.mode_callback,
			10)
		self.mover_publisher = self.create_publisher(
			String,
			'/control/mover',
			1)

		self.cwd = cwd

		self.movers = {
			0: 'pixels',
			1: 'pid',
			2: 'special'
		}
		self.curr_mover = 0

	def mode_callback(self, msg):
		curr_mode = int(msg.data)
		mover_msg = String()
		if curr_mode in [1, 4, 6:]
			mover_msg.data = '1'
		else:
			mover_msg.data = '0'
		self.mover_publisher.publish(mover_msg)


def main(args=None):
	cwd = os.getcwd()
	rclpy.init(args=args)
	try:
		controller = Controller(cwd)
		rclpy.spin(controller)
	except KeyboardInterrupt:
		controller.destroy_node()
		rclpy.shutdown()
