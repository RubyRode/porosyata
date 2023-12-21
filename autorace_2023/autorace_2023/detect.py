import os
import numpy as np

from cv_bridge import CvBridge
import cv2
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import time
from rclpy.parameter import Parameter

# sift
def generate_keypoints(pngs):
	sift = cv2.SIFT_create()
	kps = []
	# /home/ruby/ros2_ws/src/porosyata/autorace_2023/images/templates/left_.png
	for i in pngs:
		sign_pth = os.path.join(get_package_share_directory('autorace_2023'), 'images', 'templates', i)
		print(sign_pth)
		sign = cv2.imread(sign_pth)
		sign_gray = cv2.cvtColor(sign, cv2.COLOR_BGR2GRAY)
		skp, sdes = sift.detectAndCompute(sign_gray, None)
		name = i.split('.')[0]
		kps.append((skp, sdes, name))
	return kps

class Detector(Node):
	def __init__(self, cwd, signs):
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

		# self.special_callback = self.create_subscription(
		# 	Int8,
		# 	'/control/special/callback',
		# 	self.spec_call,
		# 	1)

		self.cmd_vel_publisher = self.create_publisher(
			Twist,
			"/cmd_vel",
			1)

		self.mover_publisher = self.create_publisher(
			Int8,
			'/control/mover',
			1)

		# sift
		self.sift = cv2.SIFT_create()
		self.bf = cv2.BFMatcher()
		self.signs = signs
		#

		
		# Флаг, нашли ли то, что искали
		self.is_found = False
		# Режимы работы детектора
		self.modes = {
			0: 'looking for some green lantern',
			1: 'enable PID',
			2: '/templates/intersection.png',
			3: 'found intersection sign',
			4: 'disable PID',
			5: '/templates/right_small.png', #'/templates/turn_right.png'
			6: '/templates/left_.png',
			7: 'enable PID'
			# 0: 'looking for some green lantern',
			# 1: '/templates/intersection.png',
			# 2: '/templates/turn_left.png',
			# 3: '/templates/turn_right.png',
			# 4: '/templates/blocks.png',
			# 5: 'going through some blocks',
			# 6: '/templates/parking.png',
			# 7: '/templates/car_left.png',
			# 8: '/templates/car_right.png',
 			# 9: 'ryan gosling parking',
			# 10: '/templates/zebra.png',
			# 11: 'looking for some knight',
			# 12: '/templates/tunnel.png',
			# 13: 'looking for some slam',
			# 14: 'finish',
			
		}
		# Текущий режим работы детектора
		# (сначала фиксируем зеленый свет)
		self.curr_mode = 0
		self.pid_on_off_msg = Int8()
		self.pid_on_off_msg.data = 0
		# Сообщение, которое будем публиковать при нахождении того,
		# что искали
		self.mode_msg = Int8()
		# Сообщение, которое будем получать с камеры глубины
		
		# Работа с директориями
		self.cwd = cwd
		self.images_path = self.cwd + '/src/porosyata/autorace_2023/images'
		self.curr_frame_path = self.images_path + '/curr_frame.png'
		self.curr_template = self.images_path + self.modes[self.curr_mode][0]

		self.timer_node = rclpy.create_node('intersection_timer')
		self.timer_node.set_parameters([Parameter('use_sim_time', value=True)])
		self.timer = self.timer_node.create_timer(0.0001, self.timerCb)
		self.cur_time = 0
		
		# Связка OpenCV с ROS'овскими сообщениями
		self.br = CvBridge()

	def spec_call(self, msg):
		self.curr_mode = msg.data
	
	# def mode_callback(self, msg):
	# 	curr_mode = msg.data
		
	# 	mover_msg = Int8()
	# 	if curr_mode in [1, 5, 6]:
	# 		mover_msg.data = 1
	# 	elif curr_mode in [4]:
	# 		mover_msg.data = 0
	# 	else:
	# 		mover_msg.data = -1
	# 		# special parts
	# 	# self.get_logger().info(f"control_curr_mode {curr_mode}, {mover_msg.data}")
	# 	self.mover_publisher.publish(mover_msg)

	def img_callback(self, msg):
		# Фиксируем текущий кадр
		img = self.br.imgmsg_to_cv2(msg)
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		# Записываем его (потом уберу запись, скорее всего)
		# cv2.imwrite(self.curr_frame_path, img)
		# Зеленый свет
		match self.curr_mode:
			case 0: 
				if self.detect_green(img): # looking for some green
					# self.get_logger().info("GREEN_FOUND")
					self.is_found = True # found green
					self.curr_mode = 1
			case 1:
				self.pid_on_off_msg.data = 1
				self.curr_mode = 2


				# if self.detect_sign(img):
				# 	self.pid_on_off_msg.data = -1
				# else:
				# 	self.pid_on_off_msg.data = 1

				# is_done = self.intersection()
				# if is_done:
				# 	self.curr_mode = 2
			case 2:
				if self.detect_sign(img, self.curr_mode):
					self.curr_mode = 3

			case 3:
				self.pid_on_off_msg.data = 0
				self.curr_mode = 4

			case 4:
				if self.intersection():
					self.curr_mode = 5
			case 5:
				# не удалять это сифт
				# image = self.br.imgmsg_to_cv2(msg, "bgr8") 
				# res = self.analyze(image)

				# if res == "right_small":
				# 	if self.turn_right():
				#  		self.curr_mode = 7
				# elif res == "left_":
				# 	if self.turn_left():
				# 		self.curr_mode = 7
				# if res != 'nothing':
				# 	self.direction = res
				# 	self.started = True

				# это наш детект
				if self.detect_sign(img, 5):
					if self.turn_right():
						self.curr_mode = 7
				if self.detect_sign(img, 6):
					if self.turn_left():
						self.curr_mode = 7
			case 7:
				self.pid_on_off_msg.data = 1
				# self.curr_mode = 2
			# case 2:
			# 	is_found, is_left = self.choose_from_two(img)
			# 	if is_found:
			# 		self.is_found = True
			# 		if not is_left:
			# 			self.curr_mode += 1
			# case 4:
			# 	self.detect_sign(img, self.curr_mode)
			# case 5:
			# 	# hardcode blocks
			# 	self.is_found = True
			# case 6:
			# 	self.detect_sign(img, self.curr_mode)
			# case 7:
			# 	is_found, is_left = self.choose_from_two(img)
			# 	if is_found:
			# 		self.is_found = True
			# 		if not is_left:
			# 			self.curr_mode += 1
			# case 9:
			# 	# hardcode parking
			# 	self.is_found = True
			# case 10:
			# 	self.detect_sign(img, self.curr_mode)
			# case 11:
			# 	if self.detect_pedestrian(img):
			# 		self.is_found = True
			# case 12:
			# 	self.detect_sign(img, self.curr_mode)
			# case 13:
			# 	# hardcode slam
			# 	self.is_found = True
			# case 14:
			# 	self.is_found = True

		if self.is_found:
			# self.curr_mode += 1
			self.mode_msg.data = self.curr_mode
			self.is_found = False
		self.mode_publisher.publish(self.mode_msg)
		self.mover_publisher.publish(self.pid_on_off_msg)
		self.get_logger().info(f"detect.curr_mode: {self.modes[self.curr_mode]}")


		send_img = self.br.cv2_to_imgmsg(img) 
		self.publisher.publish(send_img)
	
	def sleep(self, secs):
		rclpy.spin_once(self.timer_node)  # two times, because one is not enough to update cur_time
		rclpy.spin_once(self.timer_node)
		t0 = self.cur_time
		d = 0
	    # self.get_logger().info(f'timer started with {t0} and {secs}')
		while d <= secs:
			rclpy.spin_once(self.timer_node)
			t = self.cur_time
			d = t - t0

	def timerCb(self):
		# self.get_logger().info('timer_node callback')
		self.cur_time = self.timer_node.get_clock().now().nanoseconds * 1e-9

	def intersection(self):
		msg = Twist()
		msg.linear.x = 0.33
		msg.angular.z = 1.3
		self.cmd_vel_publisher.publish(msg)
		self.sleep(1.4)
		msg.linear.x = 0.
		msg.angular.z = 0.
		# callback_msg = Int8()
		# callback_msg.data = self.curr_mode
		self.cmd_vel_publisher.publish(msg)
		return True
		# self.special_callback.publish(callback_msg)

	def turn_right(self):# нужно скорректировать тайминги и скорости так чтобы робот норм выезжал на интерсекшн
		msg = Twist() 
		msg.linear.x = 0.3
		msg.angular.z = -1.5
		self.cmd_vel_publisher.publish(msg)
		self.sleep(2.)
		msg.linear.x = 0.7
		msg.angular.z = 0.7
		self.cmd_vel_publisher.publish(msg)
		msg.linear.x = 0.
		msg.angular.z = 0.
		self.cmd_vel_publisher.publish(msg)
		# self.sleep(0.2)
		return True

	def turn_left(self):# нужно скорректировать тайминги и скорости так чтобы робот норм выезжал на интерсекшн
		msg = Twist()
		msg.linear.x = 0.1
		msg.angular.z = 1.5
		self.cmd_vel_publisher.publish(msg)
		self.sleep(2.)
		msg.linear.x = 0.7
		msg.angular.z = -0.7
		self.cmd_vel_publisher.publish(msg)
		msg.linear.x = 0.
		msg.angular.z = 0.
		self.cmd_vel_publisher.publish(msg)
		return True

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

	def analyze(self, image):
		image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		ikp, ides = self.sift.detectAndCompute(image_gray, None)
		# BFMatcher решает матч
		for i in self.signs:
			matches = self.bf.knnMatch(i[1], ides, k=2)
			# Отрегулируйте коэффициент
			good = []
			for m, n in matches:
				if m.distance < 0.50 * n.distance:
					good.append([m])
			# sign_pth = os.path.join(get_package_share_directory('autorace_core_CVlization'), 'signs', 'left.png')
			# sign = cv2.imread(sign_pth)
			# dbg = cv2.drawMatchesKnn(sign, i[0], image, ikp, good, None, flags=2)
			# msg = self.br.cv2_to_imgmsg(dbg, 'bgr8')
			# self.dbg_pub.publish(msg)
			if len(good) >= 10:
				self.get_logger().info("intersection image analyzing " + "found " + i[2])
				return i[2]
		return 'nothing'
	
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

def main():
	cwd = os.getcwd()
	signs = generate_keypoints(["left_.png", "right_small.png"])
	rclpy.init()
	try:
		detector = Detector(cwd, signs)
		rclpy.spin(detector)
	except KeyboardInterrupt:
		detector.destroy_node()
		rclpy.shutdown()
		return 0



