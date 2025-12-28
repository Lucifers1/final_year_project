#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time
import threading

class MotorRpmControl(Node):
    def __init__(self):
        super().__init__('motor_rpm_control')

        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_name = 'base_to_bar'

        # Initial state
        self.current_angle = 0.0
        self.current_rpm = 0.0

        # 50 Hz update rate
        self.timer_period = 0.02
        self.timer = self.create_timer(self.timer_period, self.update_joint)

        # Start user input thread
        threading.Thread(target=self.get_user_input, daemon=True).start()
        self.get_logger().info("Motor RPM Control started. Enter desired RPM to rotate the bar.")

    def get_user_input(self):
        while rclpy.ok():
            try:
                rpm = float(input("Enter RPM (positive = forward, negative = reverse, 0 = stop): "))
                self.current_rpm = rpm
                self.get_logger().info(f"Set target RPM: {rpm}")
            except ValueError:
                print("Please enter a valid number.")

    def update_joint(self):
        # Convert RPM to rad/s
        angular_velocity = (self.current_rpm * 2 * math.pi) / 60.0

        # Integrate angle
        self.current_angle += angular_velocity * self.timer_period

        # Normalize angle (keep within -pi to +pi)
        self.current_angle = math.fmod(self.current_angle, 2 * math.pi)

        # Publish JointState
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.joint_name]
        msg.position = [self.current_angle]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorRpmControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

