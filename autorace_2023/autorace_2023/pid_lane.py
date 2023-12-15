import rclpy
import numpy as np
from std_msgs.msg import Float64
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ControlLane(Node):
    def __init__(self):
        super().__init__("pid_lane")

        self.sub_lane = self.create_subscription(Float64, '/detect/lane', self.cbFollowLane, 1)
        self.sub_max_vel = self.create_subscription(Float64, '/control/max_vel', self.cbGetMaxVel, 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/control/cmd_vel', 1)

        self.lastError = 0
        self.MAX_VEL = 0.1


    def cbGetMaxVel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data

    def cbFollowLane(self, desired_center):
        center = desired_center.data

        error = center - 484

        Kp = 0.0025
        Ki = 0.02
        Kd = 0.007

        angular_z = Kp * error + Ki - error + Kd * (error - self.lastError)
        self.lastError = error
        
        twist = Twist()
        # twist.linear.x = 0.05        
        twist.linear.x = min(self.MAX_VEL * ((1 - abs(error) / 484) ** 2.2), 0.05)
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        self.pub_cmd_vel.publish(twist)

    def fnShutDown(self):
        self.get_logger().info("Shutting down. cmd_vel will be 0")

        twist = Twist()
        self.pub_cmd_vel.publish(twist) 


    # def shutdown_callback():



def main():
    rclpy.init()
    node = ControlLane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.shutdown()    