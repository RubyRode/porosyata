import os
import numpy as np

from cv_bridge import CvBridge
import cv2

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8, Int8
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import time

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

		self.special_callback = self.create_publisher(
			Int8,
			'/control/special/callback',
			1)

		self.cmd_vel_publisher = self.create_publisher(
			Twist,
			"/cmd_vel",
			1)

		self.switch = 0
		self.curr_mode = 0
		self.callback = 0
		self.spec_run_rate = self.create_rate(10, self.get_clock())

	def changeMode(self, msg):
		if msg.data == -1:
			# self.get_logger().info(f"special got message: {msg.data}")
			self.switch = 1
		else:
			self.switch = 0

	def mode_callback(self, msg):
		if self.switch:

			self.curr_mode = msg.data

			match self.curr_mode:
				case 2:
					self.get_logger().info("case2")
					self.intersection()
					self.get_logger().info("case2 finished")
				case 5:
					self.blocks()
				case 9:
					self.parking()
				case 11:
					self.pedestrian()
				case 13:
					self.slam()

	def intersection(self):
		msg = Twist()
		for i in range(10):
			msg.linear.x = 0.5
			msg.angular.z = 0.5
			self.cmd_vel_publisher.publish(msg)
			time.sleep(0.1)
		msg.linear.x = 0.
		msg.angular.z = 0.
		self.curr_mode = 3
		callback_msg = Int8()
		callback_msg.data = self.curr_mode
		self.cmd_vel_publisher.publish(msg)
		self.special_callback.publish(callback_msg)

def main(args=None):
	rclpy.init(args=args)
	try:
		special = Special()
		rclpy.spin(special)
	except KeyboardInterrupt:
		special.destroy_node()
		rclpy.shutdown()
		return 0
