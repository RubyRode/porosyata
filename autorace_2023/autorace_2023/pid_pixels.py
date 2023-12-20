import os
import numpy as np

from cv_bridge import CvBridge
import cv2

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8, Int8
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Pid(Node):
	def __init__(self, cwd):
		super().__init__('pid')
		self.img_subscription = self.create_subscription(
			Image,
			'/color/image/projected',
			self.img_callback,
			10)
		self.cmd_vel_publisher = self.create_publisher(
			Twist,
			'/cmd_vel',
			1)
		self.sub_mode = self.create_subscription(
			Int8, 
			"/control/mover", 
			self.changeMode, 
			1)


		self.switch = 0
		
		# Работа с директориями
		self.cwd = cwd
		self.images_path = self.cwd + '/src/porosyata/detect_signs/images'
		
		# Связка OpenCV с ROS'овскими сообщениями
		self.br = CvBridge()

	def changeMode(self, msg):
		if msg.data == 0:
			self.switch = 1
		else:
			self.switch = 0
	
	def img_callback(self, msg):
		# Фиксируем текущий кадр
		img = self.br.imgmsg_to_cv2(msg)
		
		# Границы для цветов
		yellow_lower = np.array([0, 210, 210], dtype="uint8")
		yellow_upper = np.array([0, 255, 255], dtype="uint8")
		white_lower = np.array([250, 250, 250], dtype="uint8")
		white_upper = np.array([255, 255, 255], dtype="uint8")
		
		yellow_color = self.detect_line(img, yellow_lower, yellow_upper)
		white_color = self.detect_line(img, white_lower, white_upper)
		
		num_yellow_pixels = np.count_nonzero(yellow_color) + 1
		num_white_pixels = np.count_nonzero(white_color) + 1
		send_msg = Twist()
		woy = num_white_pixels/num_yellow_pixels
		yow = num_yellow_pixels/num_white_pixels
		alpha = 0.8
		if alpha <= woy <= woy + 20 and alpha <= yow <= yow + 20:
			send_msg.linear.x = 0.7 # forward
			# self.angular.data = 'forward'
		elif yow < alpha and woy > 20:
			send_msg.linear.x = 0.5
			send_msg.angular.z = -0.8
			# self.angular.data = 'left'
		
		if num_white_pixels/num_yellow_pixels > 6:
			self.angular.data = 'left'
		elif num_yellow_pixels/num_white_pixels > 6:
			self.angular.data = 'right'
		else:
			send_msg.linear.x = 0.5
			send_msg.angular.z = 0.8
			# self.angular.data = 'right'
		
		# print(num_white_pixels/num_yellow_pixels)
		if self.switch == 1:
			self.cmd_vel_publisher.publish(send_msg)
		# self.angular_publisher.publish(angular)
		
		# whole_color = cv2.bitwise_or(yellow_color, white_color)
		# cv2.rectangle(whole_color, (100, 100), (200, 200), signal_color, 2)
		
		# cv2.imshow('colors', whole_color)
		# cv2.waitKey(1)
	
	def detect_line(self, img, lower, upper):
		color_mask = cv2.inRange(img, lower, upper)
		img_masked = cv2.bitwise_and(img, img, mask=color_mask)
		return img_masked
			
		

def main(args=None):
	cwd = os.getcwd()
	rclpy.init(args=args)
	try:
		pid = Pid(cwd)
		rclpy.spin(pid)
	except KeyboardInterrupt:
		pid.destroy_node()
		rclpy.shutdown()
		return 0

if __name__ == '__main__':
    main()

