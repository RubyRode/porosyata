import os
import numpy as np

from cv_bridge import CvBridge
import cv2

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class PixelMover(Node):
	def __init__(self):
		super().__init__('pixel_mover')
		self.control_subscription = self.create_subscription(
			String,
			'/control/mover',
			self.control_callback,
			10)
		self.pixel_subscription = self.create_subscription(
			String,
			'/detect/pid_angular',
			self.pixel_callback,
			10)
		self.cmd_vel_publisher = self.create_publisher(
			Twist,
			'/cmd_vel',
			1)

		# Требуется ли сейчас использовать
		# движение по пикселям
		self.is_working = False

	def control_callback(self, msg):
		if int(msg.data) == 0:
			self.is_working = True
		else:
			self.is_working = False

	def pixel_callback(self, msg):
		if self.is_working:
			curr_direction = msg.data

			cmd_vel_msg = Twist()
			cmd_vel_msg.linear.x = 0.1

			if curr_direction == 'right':
				cmd_vel_msg.angular.z = -0.1
			elif curr_direction == 'left':
				cmd_vel_msg.angular.z = 0.1

			self.cmd_vel_publisher.publish(cmd_vel_msg)

def main(args=None):
	rclpy.init(args=args)
	try:
		pixel_mover = PixelMover()
		rclpy.spin(pixel_mover)
	except KeyboardInterrupt:
		pixel_mover.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
    main()


