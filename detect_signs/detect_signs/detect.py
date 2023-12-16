import os
import numpy as np

from cv_bridge import CvBridge
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class Detector(Node):
	def __init__(self, cwd):
		super().__init__('snap_subscriber')
		self.subscription = self.create_subscription(
			Image,
			'/color/image',
			self.snap_callback,
			10)
		self.subscription
		
		# Связка OpenCV с ROS'овскими сообщениями
		self.br = CvBridge()
		
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
			14: 'finish'
			
		}
		# Текущий режим работы детектора
		# (сначала фиксируем зеленый свет)
		self.curr_mode = 0
		
		# Работа с директориями
		self.cwd = cwd
		self.images_path = self.cwd + '/src/porosyata/detect_signs/images'
		self.curr_frame_path = self.images_path + '/curr_frame.png'
		self.curr_template = self.images_path + self.modes[self.curr_mode]
	
	def snap_callback(self, msg):
		# Фиксируем текущий кадр
		img = self.br.imgmsg_to_cv2(msg)
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		# Записываем его (потом уберу запись, скорее всего)
		cv2.imwrite(self.curr_frame_path, img)
		
		# Зеленый свет
		if self.curr_mode == 0:
			self.detect_green(img)
		# Выбор направления поворота или места на парковке
		# (выбор между двумя вариантами)
		elif self.curr_mode == 2 or self.curr_mode == 7:
			self.choose_from_two(img)
		elif self.curr_mode == 5 or self.curr_mode == 9 or self.curr_mode == 11:
			self.curr_mode += 1
		# Просто поиск знаков
		else:
			self.detect_sign(img, self.curr_mode)
		
		# Если нашли, что искали, переключаем режим
		if self.is_found:
			self.curr_mode += 1
			self.curr_mode %= 14
			self.curr_template = self.images_path + self.modes[self.curr_mode]
			self.is_found = False
		
		print(self.curr_mode)
		cv2.imshow('camera', img)
		cv2.waitKey(1)
	
	def detect_green(self, img):
		lower = np.array([0, 40, 0], dtype="uint8")
		upper = np.array([10, 255, 10], dtype="uint8")
		green_mask = cv2.inRange(img, lower, upper)
		img_masked = cv2.bitwise_and(img, img, mask=green_mask)
		
		if np.any(img_masked[279, 605] != 0):
			self.is_found = True
	
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
			self.curr_mode += 1
		

def main(args=None):
	cwd = os.getcwd()
	rclpy.init(args=args)
	
	detector = Detector(cwd)
	rclpy.spin(detector)
	
	detector.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

