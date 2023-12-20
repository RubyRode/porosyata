import rclpy
import numpy as np
from std_msgs.msg import Float64, String, UInt8, Int8
from rclpy.node import Node
from geometry_msgs.msg import Twist
from autorace_msgs.msg import PidGraph
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, ParameterType

class ControlLane(Node):
    def __init__(self):
        super().__init__("pid_lane")

        self.sub_lane = self.create_subscription(Float64, '/detect/lane', self.cbFollowLane, 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        # self.pub_plot_error = self.create_publisher(PidGraph, '/detect/plot', 1)
        self.sub_mode = self.create_subscription(Int8, "/control/mover", self.changeMode, 1)

        self.K_p_p = self.declare_parameter("K_p", 50, 
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,
                integer_range=[IntegerRange(from_value= 5, to_value=10000, step=5)])).value
        self.K_i_p = self.declare_parameter("K_i", 10, 
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,
                integer_range=[IntegerRange(from_value= 10, to_value=10000, step=10)])).value
        self.K_d_p = self.declare_parameter("K_d", 500, 
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,
                integer_range=[IntegerRange(from_value= 10, to_value=10000, step=10)])).value
        self.MAX_VEL_p = self.declare_parameter("MAX_VEL", 3, 
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,
                integer_range=[IntegerRange(from_value=0, to_value=10, step=1)])).value
        self.MAX_ANG_p = self.declare_parameter("MAX_ANG", 5, 
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,
                integer_range=[IntegerRange(from_value=0, to_value=10, step=1)])).value

        self.K_p = self.get_parameter("K_p").get_parameter_value().integer_value / 10000
        self.K_i = self.get_parameter("K_i").get_parameter_value().integer_value / 10000
        self.K_d = self.get_parameter("K_d").get_parameter_value().integer_value / 10000
        self.lastError = 0
        self.I = 0
        self.t0 = self.get_clock().now().nanoseconds * (10**(-9))
        self.max_ang = 1
        self.MAX_VEL = self.get_parameter("MAX_VEL").get_parameter_value().integer_value / 10
        self.MAX_ANG = self.get_parameter("MAX_ANG").get_parameter_value().integer_value / 10
        self.switch = 0 


    def changeMode(self, msg):
        if msg.data == 1:

            self.switch = 1
        else:
            self.I = 0
            self.switch = 0

    def cbFollowLane(self, desired_center):
        if self.switch == 1:
            self.K_p = self.get_parameter("K_p").get_parameter_value().integer_value / 10000
            self.K_i = self.get_parameter("K_i").get_parameter_value().integer_value / 10000
            self.K_d = self.get_parameter("K_d").get_parameter_value().integer_value / 10000
            self.MAX_VEL = self.get_parameter("MAX_VEL").get_parameter_value().integer_value / 10
            self.MAX_ANG = self.get_parameter("MAX_ANG").get_parameter_value().integer_value / 10
            center = desired_center.data

            error = center - 424
            
            Kp = self.K_p
            Kd = self.K_d
            Ki = self.K_i
            dt = self.get_clock().now().nanoseconds * (10**(-9)) - self.t0 + 0.0000001
            e_i = self.I/dt

            angular_z = Kp * error + Kd * (error - self.lastError) + Ki * e_i
            self.lastError = error
            self.I += error

            twist = Twist()

            twist.linear.x = min(self.MAX_VEL * (abs(1 - abs(error) / 424) ** 2), 0.8)

            twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
            self.pub_cmd_vel.publish(twist)

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
        return 0
