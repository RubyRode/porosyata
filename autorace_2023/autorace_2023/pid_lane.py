import rclpy
import numpy as np
from std_msgs.msg import Float64
from rclpy.node import Node
from geometry_msgs.msg import Twist
from autorace_msgs.msg import TimeStampedFloat64

class ControlLane(Node):
    def __init__(self):
        super().__init__("pid_lane")

        self.sub_lane = self.create_subscription(TimeStampedFloat64, '/detect/lane', self.cbFollowLane, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 1)

        self.declare_parameters(
            namespace='',
            parameters=[
            ("K_p", 0.0025),
            ("K_d", 0.007),
            ("MAX_VEL", 0.5),
            ("MAX_ANG", 0.5),
        ])


        self.K_p = self.get_parameter("K_p").get_parameter_value().double_value
        self.K_d = self.get_parameter("K_d").get_parameter_value().double_value
        self.lastError = 0

        self.MAX_VEL = self.get_parameter("MAX_VEL").get_parameter_value().double_value
        self.MAX_ANG = self.get_parameter("MAX_ANG").get_parameter_value().double_value


    # def cbGetMaxVel(self, max_vel_msg):
    #     self.MAX_VEL = max_vel_msg.data

    def cbFollowLane(self, desired_center):
        self.K_p = self.get_parameter("K_p").get_parameter_value().double_value
        self.K_d = self.get_parameter("K_d").get_parameter_value().double_value
        self.MAX_VEL = self.get_parameter("MAX_VEL").get_parameter_value().double_value
        self.MAX_ANG = self.get_parameter("MAX_ANG").get_parameter_value().double_value
        center = desired_center.data

        # self.get_logger().info(f"{center}")
        error = center - 484


        angular_z = self.K_p * error + self.K_d * (error - self.lastError)
        # self.get_logger().info(f"{angular_z}")
        self.lastError = error
        try:
            twist = Twist()
            # twist.linear.x = 0.05   
            # self.get_logger().info(f"{self.MAX_VEL * ((1 - abs(error) / 484) ** 2.2)}")
            linear_x = float(self.MAX_VEL * ((1 - abs(error) / 484) ** 2))
            # self.get_logger().info(f"{linear_x}")   
            twist.linear.x = linear_x
            twist.linear.y = 0.
            twist.linear.z = 0.
            twist.angular.x = 0.
            twist.angular.y = 0.
            twist.angular.z = -max(angular_z * self.MAX_ANG, -2.0) if angular_z < 0 else -min(angular_z * self.MAX_ANG, 2.0)
            self.pub_cmd_vel.publish(twist)
        except TypeError as e:
            self.get_logger().info(f"{self.MAX_VEL * ((1 - abs(error) / 484) ** 2.2)}")
            self.get_logger().info(f"{self.MAX_VEL}, {abs(error)}")
            self.fnShutDown()

    def fnShutDown(self):
        self.get_logger().info("Shutting down. cmd_vel will be 0")

        twist = Twist()
        self.pub_cmd_vel.publish(twist)
        self.destroy_node() 
        rclpy.shutdown()





def main():
    rclpy.init()
    node = ControlLane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.fnShutDown()
