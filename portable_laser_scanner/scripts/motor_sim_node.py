#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class MotorSim(Node):
    def __init__(self):
        super().__init__('motor_sim')
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_name = 'base_to_bar'

        # motor characteristics
        self.pwm_value = 128   # change this to simulate speed
        self.rpm = 0.0823 * self.pwm_value
        self.angular_velocity = (self.rpm * 2 * math.pi) / 60  # rad/s

        self.angle = 0.0
        self.timer = self.create_timer(0.02, self.update_joint)  # 50 Hz
        self.get_logger().info(f"MotorSim started: {self.rpm:.2f} RPM ({self.angular_velocity:.3f} rad/s)")

    def update_joint(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.joint_name]
        self.angle += self.angular_velocity * 0.02  # integrate
        msg.position = [self.angle]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
