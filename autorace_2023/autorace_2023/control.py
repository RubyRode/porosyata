import os
import numpy as np

from cv_bridge import CvBridge
import cv2

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8, Int8
from sensor_msgs.msg import Image

class Controller(Node):
	def __init__(self, cwd):
		super().__init__('controller')
		self.mode_subscriber = self.create_subscription(
			Int8,
			'/detect/signs/mode',
			self.mode_callback,
			10)
		self.mover_publisher = self.create_publisher(
			Int8,
			'/control/mover',
			1)

		self.curr_mover = 0

	def mode_callback(self, msg):
		curr_mode = msg.data
		self.get_logger().info(f"control_curr_mode {curr_mode}")
		mover_msg = Int8()
		if curr_mode in [1, 4, 6]:
			mover_msg.data = 1
		elif curr_mode in [2]:
			mover_msg.data = 0
		else:
			mover_msg.data = -1
		self.mover_publisher.publish(mover_msg)
		# self.get_logger().info(f'/control/mover :{mover_msg.data}')


def main(args=None):
	cwd = os.getcwd()
	rclpy.init(args=args)
	try:
		controller = Controller(cwd)
		rclpy.spin(controller)
	except KeyboardInterrupt:
		controller.destroy_node()
		rclpy.shutdown()
