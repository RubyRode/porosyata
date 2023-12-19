import os
import numpy as np

from cv_bridge import CvBridge
import cv2

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8, Int8
from sensor_msgs.msg import Image

class Detector(Node):
	def __init__(self, cwd):
		super().__init__('snap_subscriber')
		self.img_subscription = self.create_subscription(
			Image,
			'/color/image',
			self.img_callback,
			10)
		self.publisher = self.create_publisher(
			Image,
			"/detect/signs",
			1)
		self.mode_publisher = self.create_publisher(
			Int8, 
			'/detect/signs/mode', 
			1)
		
		# Флаг, нашли ли то, что искали
		self.is_found = False
		# Режимы работы детектора
		self.modes = {
			0: 'looking for some green lantern',
			1: '/templates/intersection.png',
			2: '/templates/turn_left.png',
			3: '/templates/turn_right.png',
			4: '/templates/blocks.png',
			5: 'going through some blocks',
			6: '/templates/parking.png',
			7: '/templates/car_left.png',
			8: '/templates/car_right.png',
 			9: 'ryan gosling parking',
			10: '/templates/zebra.png',
			11: 'looking for some knight',
			12: '/templates/tunnel.png',
			13: 'looking for some slam',
			14: 'finish',
			
		}
		# Текущий режим работы детектора
		# (сначала фиксируем зеленый свет)
		self.curr_mode = 0
		# Сообщение, которое будем публиковать при нахождении того,
		# что искали
		self.mode_msg = Int8()
		# Сообщение, которое будем получать с камеры глубины
		
		# Работа с директориями
		self.cwd = cwd
		self.images_path = self.cwd + '/src/porosyata/autorace_2023/images'
		self.curr_frame_path = self.images_path + '/curr_frame.png'
		self.curr_template = self.images_path + self.modes[self.curr_mode][0]
		
		# Связка OpenCV с ROS'овскими сообщениями
		self.br = CvBridge()
	
	def img_callback(self, msg):
		# Фиксируем текущий кадр
		img = self.br.imgmsg_to_cv2(msg)
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		# Записываем его (потом уберу запись, скорее всего)
		# cv2.imwrite(self.curr_frame_path, img)
		
		# Зеленый свет
		match self.curr_mode:
			case 0: 
				if self.detect_green(img):
					self.is_found = True
			case 1:
				self.detect_sign(img, self.curr_mode)
			case 2:
				is_found, is_left = self.choose_from_two(img)
				if is_found:
					self.is_found = True
					if not is_left:
						self.curr_mode += 1
			case 4:
				self.detect_sign(img, self.curr_mode)
			case 5:
				# hardcode blocks
				self.is_found = True
			case 6:
				self.detect_sign(img, self.curr_mode)
			case 7:
				is_found, is_left = self.choose_from_two(img)
				if is_found:
					self.is_found = True
					if not is_left:
						self.curr_mode += 1
			case 9:
				# hardcode parking
				self.is_found = True
			case 10:
				self.detect_sign(img, self.curr_mode)
			case 11:
				if self.detect_pedestrian(img):
					self.is_found = True
			case 12:
				self.detect_sign(img, self.curr_mode)
			case 13:
				# hardcode slam
				self.is_found = True
			case 14:
				self.is_found = True

		if self.is_found:
			self.curr_mode += 1
			self.mode_msg.data = self.curr_mode
			self.is_found = False

		self.mode_publisher.publish(self.mode_msg)
		#self.get_logger().info(f"/detect/signs/mode: {self.mode_msg.data}, {self.curr_mode}")



		# if self.curr_mode == 0:
		# 	self.detect_green(img)
		# # Выбор направления поворота или места на парковке
		# # (выбор между двумя вариантами)
		# elif self.curr_mode == 2 or self.curr_mode == 7:
		# 	self.choose_from_two(img)
		# # Ищем пешехода перед машиной
		# elif self.curr_mode == 11:
		# 	self.detect_pedestrian(img)
		# # Заглушка
		# elif self.curr_mode == 5 or self.curr_mode == 9:
		# 	self.curr_mode += 1
		# # Просто поиск знаков
		# else:
		# 	self.detect_sign(img, self.curr_mode)
		# # self.get_logger().info(f"{self.curr_mode}")
		# print(self.modes[self.curr_mode])
		# # Если нашли, что искали, переключаем режим
		# if self.is_found:
		# 	self.mode_publisher.publish(self.mode_msg)
		# 	self.curr_mode += 1
		# 	self.curr_mode %= 14
		# 	self.curr_template = self.images_path + self.modes[self.curr_mode]
		# 	self.is_found = False

		send_img = self.br.cv2_to_imgmsg(img) 
		self.publisher.publish(send_img)
		
	
	def detect_green(self, img):
		lower = np.array([0, 40, 0], dtype="uint8")
		upper = np.array([10, 255, 10], dtype="uint8")
		green_mask = cv2.inRange(img, lower, upper)
		img_masked = cv2.bitwise_and(img, img, mask=green_mask)
		# cv2.imshow("sudhfkjsd", img_masked)
		# cv2.waitKey(1)
		if np.any(img_masked[279, 605] != 0):
			return True

		return False
	
	def detect_sign(self, img, mode):
		img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
		template = cv2.imread(self.images_path + self.modes[mode], 0)
		
		w, h = template.shape
		res = cv2.matchTemplate(img_gray, template, cv2.TM_CCOEFF_NORMED)
		threshold = 0.8
		loc = np.where(res >= threshold)
		for pt in zip(*loc[::-1]):
			if not self.is_found:
				self.is_found = True
			cv2.rectangle(img, pt, (pt[0] + w, pt[1] + h), (255, 0, 150), 2)
		return self.is_found
	
	def choose_from_two(self, img):
		is_left = False
		is_right = False
		
		is_left = self.detect_sign(img, self.curr_mode)
		is_right = self.detect_sign(img, self.curr_mode + 1)
		
		if is_left or is_right:
			if is_left:
				self.mode_msg.data = self.curr_mode
			else:
				self.mode_msg.data = self.curr_mode + 1
			return True, is_left

		return False, False
	
	def detect_pedestrian(self, img):
		row = 99
		left = 330
		right = 520
		
		if np.all(img[row, 330:520] == img[row, 330]):
			self.mode_msg.data = self.curr_mode
			return True

		return False

def main(args=None):
	cwd = os.getcwd()
	rclpy.init(args=args)
	try:
		detector = Detector(cwd)
		rclpy.spin(detector)
	except KeyboardInterrupt:
		detector.destroy_node()
		rclpy.shutdown()



