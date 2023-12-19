import os
import numpy as np

from cv_bridge import CvBridge
import cv2

import rclpy
from rclpy.node import Node
<<<<<<< HEAD
from std_msgs.msg import String, UInt8, Int8
=======
from std_msgs.msg import String
>>>>>>> 2f6bc480e01e5b6c1bde471b4555c10cf351eb6e
from sensor_msgs.msg import Image

class Controller(Node):
	def __init__(self, cwd):
		super().__init__('controller')
		self.mode_subscriber = self.create_subscription(
<<<<<<< HEAD
			Int8,
=======
			String,
>>>>>>> 2f6bc480e01e5b6c1bde471b4555c10cf351eb6e
			'/detect/signs/mode',
			self.mode_callback,
			10)
		self.mover_publisher = self.create_publisher(
<<<<<<< HEAD
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
=======
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
>>>>>>> 2f6bc480e01e5b6c1bde471b4555c10cf351eb6e


def main(args=None):
	cwd = os.getcwd()
	rclpy.init(args=args)
	try:
		controller = Controller(cwd)
		rclpy.spin(controller)
	except KeyboardInterrupt:
		controller.destroy_node()
		rclpy.shutdown()
