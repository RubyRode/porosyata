import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from PIL import Image as Pilimage
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

def find_coeffs(pa, pb):
    matrix = []
    for p1, p2 in zip(pa, pb):
        matrix.append([p1[0], p1[1], 1, 0, 0, 0, -p2[0]*p1[0], -p2[0]*p1[1]])
        matrix.append([0, 0, 0, p1[0], p1[1], 1, -p2[1]*p1[0], -p2[1]*p1[1]])

    A = np.matrix(matrix, dtype=np.float32)
    B = np.array(pb).reshape(8)

    res = np.dot(np.linalg.inv(A.T * A) * A.T, B)
    return np.array(res).reshape(8)

class ImageProjection(Node):
    def __init__(self):
        super().__init__('image_projection')

        self.declare_parameters(
            namespace='',
            parameters=[
            ('top_x', 72),
            ('top_y', 4),
            ('bottom_x', 115),
            ('bottom_y', 120),
            ('left_botom_x', 39),
            ('left_botom_y', 479),
            ('left_top_x', 281),
            ('left_top_y', 246),
            ('right_top_x', 539),
            ('right_top_y', 246),
            ('right_botom_x', 743),
            ('right_botom_y', 479),
        ])

        self.sub_image_type = "raw"        # "compressed" / "raw"
        self.pub_image_type = "raw"        # "compressed" / "raw"


        self.sub_image_compensated = self.create_subscription(Image, '/color/image', self.cbImageProjection, 1)

 
        self.pub_image_projected = self.create_publisher(Image, '/color/image/projected', 1)



        self.cvBridge = CvBridge()

    def cbImageProjection(self, msg_img):
        # self.top_x = self.get_parameter("top_x").get_parameter_value().integer_value
        # self.top_y = self.get_parameter("top_y").get_parameter_value().integer_value
        # self.bottom_x = self.get_parameter("bottom_x").get_parameter_value().integer_value
        # self.bottom_y = self.get_parameter("bottom_y").get_parameter_value().integer_value

        # if self.sub_image_type == "compressed":
        #     # converts compressed image to opencv image
        #     np_image_original = np.frombuffer(msg_img.data, np.uint8)
        #     cv_image_original = cv2.imdecode(np_image_original, cv2.IMREAD_COLOR)
        # elif self.sub_image_type == "raw":
        #     # converts raw image to opencv image
        #     cv_image_original = self.cvBridge.imgmsg_to_cv2(msg_img, "bgr8")

        # # setting homography variables
        # top_x = self.top_x
        # top_y = self.top_y
        # bottom_x = self.bottom_x
        # bottom_y = self.bottom_y

        # if self.calib == True:
        #     # copy original image to use for cablibration
        #     cv_image_calib = np.copy(cv_image_original)

        #     # draw lines to help setting homography variables
        #     cv_image_calib = cv2.line(cv_image_calib, (424 - top_x, 240 - top_y), (424 + top_x, 240 - top_y), (0, 0, 255), 1)
        #     cv_image_calib = cv2.line(cv_image_calib, (424 - bottom_x, 240 + bottom_y), (424 + bottom_x, 240 + bottom_y), (0, 0, 255), 1)
        #     cv_image_calib = cv2.line(cv_image_calib, (424 + bottom_x, 240 + bottom_y), (424 + top_x, 240 - top_y), (0, 0, 255), 1)
        #     cv_image_calib = cv2.line(cv_image_calib, (424 - bottom_x, 240 + bottom_y), (424 - top_x, 240 - top_y), (0, 0, 255), 1)

        #     if self.pub_image_type == "compressed":
        #         # publishes calibration image in compressed type
        #         self.pub_image_calib.publish(self.cvBridge.cv2_to_compressed_imgmsg(cv_image_calib, "jpg"))

        #     elif self.pub_image_type == "raw":
        #         # publishes calibration image in raw type
        #         self.pub_image_calib.publish(self.cvBridge.cv2_to_imgmsg(cv_image_calib, "bgr8"))

        # # adding Gaussian blur to the image of original
        # cv_image_original = cv2.GaussianBlur(cv_image_original, (5, 5), 0)

        # ## homography transform process
        # # selecting 4 points from the original image
        # pts_src = np.array([[424 - top_x, 240 - top_y], [424 + top_x, 240 - top_y], [424 + bottom_x, 240 + bottom_y], [424 - bottom_x, 240 + bottom_y]])

        # # selecting 4 points from image that will be transformed
        # pts_dst = np.array([[100, 0], [848, 0], [848, 480], [100, 480]])

        # # finding homography matrix
        # h, status = cv2.findHomography(pts_src, pts_dst)

        # # homography process
        # cv_image_homography = cv2.warpPerspective(cv_image_original, h, (848, 480))

        # fill the empty space with black triangles on left and right side of bottom
        # triangle1 = np.array([[0, 479], [0, 260], [148, 479]], np.int32)
        # triangle2 = np.array([[847, 479], [847, 260], [647, 479]], np.int32)
        # black = (0, 0, 0)
        # white = (255, 255, 255)
        # cv_image_homography = cv2.fillPoly(cv_image_homography, [triangle1, triangle2], black)
        x0 = self.get_parameter('left_botom_x').get_parameter_value().integer_value
        y0 = self.get_parameter('left_botom_y').get_parameter_value().integer_value
        x1 = self.get_parameter('left_top_x').get_parameter_value().integer_value
        y1 = self.get_parameter('left_top_y').get_parameter_value().integer_value
        x2 = self.get_parameter('right_top_x').get_parameter_value().integer_value
        y2 = self.get_parameter('right_top_y').get_parameter_value().integer_value
        x3 = self.get_parameter('right_botom_x').get_parameter_value().integer_value
        y3 = self.get_parameter('right_botom_y').get_parameter_value().integer_value
        self

        img = self.cvBridge.imgmsg_to_cv2(msg_img, "bgr8")
        img = Pilimage.fromarray(img)
        orig = [[0, 479], [0, 0], [847, 0], [847, 479]]
        new = [[x0, y0], [x1, y1], [x2, y2], [x3, y3]]
        cfs = find_coeffs(orig, new)
        cv_image_homography = img.transform(img.size, 2, data=cfs, resample=Pilimage.BILINEAR)
        cv_image_homography = np.asarray(cv_image_homography)
        

        self.pub_image_projected.publish(self.cvBridge.cv2_to_imgmsg(cv_image_homography, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = ImageProjection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()