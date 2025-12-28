#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class MotorAngleControl(Node):
    def __init__(self):
        super().__init__('motor_angle_control')
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_name = 'base_to_bar'
        self.current_angle = 0.0
        self.timer = self.create_timer(0.02, self.update_joint)  # 50 Hz

        self.target_angle = 0.0
        self.speed = 1.0  # rad per second
        self.moving = False

        self.get_logger().info("MotorAngleControl started. Type angles in degrees to move the bar.")

        # separate thread for user input
        import threading
        threading.Thread(target=self.read_input, daemon=True).start()

    def read_input(self):
        while rclpy.ok():
            try:
                val = float(input("Enter target angle (degrees): "))
                self.target_angle = math.radians(val)
                self.moving = True
            except Exception as e:
                print("Invalid input:", e)

    def update_joint(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.joint_name]

        if self.moving:
            diff = self.target_angle - self.current_angle
            step = self.speed * 0.02  # per timer step (50 Hz)
            if abs(diff) < step:
                self.current_angle = self.target_angle
                self.moving = False
            else:
                self.current_angle += step if diff > 0 else -step

        msg.position = [self.current_angle]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorAngleControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
