#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # 1. Setup Publishers
        # Note: The topic name MUST match what is in the Launch file bridge
        self.pitch_pub = self.create_publisher(
            Float64, 
            '/model/motor_test_robot/joint/bar_to_semi_octagon/cmd_pos', 
            10)
        
        self.yaw_pub = self.create_publisher(
            Float64, 
            '/model/motor_test_robot/joint/base_to_bar/cmd_pos', 
            10)

        self.get_logger().info("Robot Controller Started. Waiting 2s...")
        time.sleep(2) # Wait for connections

    def send_pitch(self, angle_degrees):
        msg = Float64()
        msg.data = math.radians(angle_degrees)
        self.pitch_pub.publish(msg)
        self.get_logger().info(f"Pitching to {angle_degrees}°")

    def send_yaw(self, angle_degrees):
        msg = Float64()
        msg.data = math.radians(angle_degrees)
        self.yaw_pub.publish(msg)
        self.get_logger().info(f"Yawing to {angle_degrees}°")

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()

    try:
        # --- SEQUENCE START ---
        
        # 1. Move -45 degrees (Pitch)
        controller.send_pitch(-45.0)
        time.sleep(3) # Wait for movement

        # 2. Move 90 degrees (Pitch)
        controller.send_pitch(90.0)
        time.sleep(3)

        # 3. Move Base 90 degrees Left (Yaw)
        controller.send_yaw(90.0)
        time.sleep(3)

        # 4. Repeat Step 1 (Pitch -45)
        controller.send_pitch(-45.0)
        time.sleep(3)

        # 5. Repeat Step 2 (Pitch 90)
        controller.send_pitch(90.0)
        time.sleep(3)

        controller.get_logger().info("Sequence Complete.")

    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
