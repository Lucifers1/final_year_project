#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class RPMController(Node):
    def __init__(self):
        super().__init__("rpm_controller")

        self.pub = self.create_publisher(
            Float64,
            "/base_to_bar/velocity_controller/command",
            10
        )

        self.get_logger().info("Enter RPM values…")

        self.timer = self.create_timer(0.1, self.update)

    def update(self):
        rpm = float(input("Enter RPM: "))
        rad_s = rpm * 2 * 3.14159 / 60.0

        msg = Float64()
        msg.data = rad_s

        self.pub.publish(msg)
        print(f"Sent angular speed: {rad_s:.3f} rad/s")

def main():
    rclpy.init()
    node = RPMController()
    rclpy.spin(node)

if __name__ == "__main__":
    main()

