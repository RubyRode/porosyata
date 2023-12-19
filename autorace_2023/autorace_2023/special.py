import os
import numpy as np

from cv_bridge import CvBridge
import cv2

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8, Int8
from std_msgs.msg import String
from sensor_msgs.msg import Image

class Special(Node):
	def __init__(self):
		super().__init__('special')

		self.mode_subscription = self.create_subscription(
			Int8,
			'/detect/signs/mode',
			self.mode_callback, 
			1)
		self.mover_subscription = self.create_subscription(
			Int8,
			'/control/mover',
			self.changeMode,
			1)

		self.switch = 0
		self.curr_mode = 0

	def changeMode(self, msg):
		if msg == -1:
			self.switch = 1
		else:
			self.switch = 0

	def mode_callback(self, msg):
		if self.switch:
			self.curr_mode = msg

			match self.curr_mode:
				case 5:
					self.blocks()
				case 9:
					self.parking()
				case 11:
					self.pedestrian()
				case 13:
					self.slam()



def main(args=None):
	rclpy.init(args=args)
	try:
		special = Special()
		rclpy.spin(special)
	except KeyboardInterrupt:
		special.destroy_node()
		rclpy.shutdown()
