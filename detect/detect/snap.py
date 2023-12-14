import os
import numpy as np

from cv_bridge import CvBridge
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class SnapSubscriber(Node):
	def __init__(self, cwd):
		super().__init__('snap_subscriber')
		self.subscription = self.create_subscription(
			Image,
			'/color/image',
			self.snap_callback,
			10)
		self.subscription
		
		self.br = CvBridge()
		
		self.cwd = cwd
		self.images_path = self.cwd + '/src/porosyata/detect/images'
		self.curr_frame_path = self.images_path + '/curr_frame.png'
		self.intersection_path = self.images_path + '/templates/intersection.png'
	
	def snap_callback(self, msg):
		img = self.br.imgmsg_to_cv2(msg)
		img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
		template = cv2.imread(self.intersection_path, 0)
		
		w, h = template.shape
		res = cv2.matchTemplate(img_gray, template, cv2.TM_CCOEFF_NORMED)
		threshold = 0.8
		loc = np.where(res >= threshold)
		for pt in zip(*loc[::-1]):
			cv2.rectangle(img, pt, (pt[0] + w, pt[1] + h), (0, 255, 255), 2)
		
		cv2.imshow('camera', img)
		cv2.imwrite(self.curr_frame_path, img)
		cv2.waitKey(1)

def main(args=None):
	cwd = os.getcwd()
	rclpy.init(args=args)
	
	snap_subscriber = SnapSubscriber(cwd)
	rclpy.spin(snap_subscriber)
	
	snap_subscriber.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

