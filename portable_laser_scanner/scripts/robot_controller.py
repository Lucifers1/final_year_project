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
        self.pitch_pub = self.create_publisher(
            Float64, 
            '/model/motor_test_robot/joint/bar_to_semi_octagon/cmd_pos', 
            10)
        
        self.yaw_pub = self.create_publisher(
            Float64, 
            '/model/motor_test_robot/joint/base_to_bar/cmd_pos', 
            10)

        # 2. Track current positions (Assume we start at 0.0)
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        
        # Control Loop Frequency (Hz) - Higher is smoother
        self.rate = 50.0 
        self.dt = 1.0 / self.rate

        self.get_logger().info("Robot Controller Started. Waiting 2s...")
        time.sleep(2)

    def move_smoothly(self, joint_type, target_angle_deg, rpm):
        """
        Moves a joint from current position to target position at a specific RPM.
        joint_type: 'pitch' or 'yaw'
        target_angle_deg: Target angle in degrees
        rpm: Speed in Revolutions Per Minute
        """
        
        # Determine start angle and publisher based on joint type
        if joint_type == 'pitch':
            start_angle = self.current_pitch
            publisher = self.pitch_pub
        else:
            start_angle = self.current_yaw
            publisher = self.yaw_pub

        # --- MATH TIME ---
        # 1. Convert RPM to Degrees per Second
        #    (RPM * 360) / 60  =>  RPM * 6
        speed_deg_per_sec = rpm * 6.0
        
        # 2. Calculate distance to travel
        distance = target_angle_deg - start_angle
        
        # 3. Calculate time required (Duration = Distance / Speed)
        if speed_deg_per_sec <= 0:
            duration = 0
        else:
            duration = abs(distance) / speed_deg_per_sec

        # 4. Calculate number of steps
        steps = int(duration * self.rate)
        
        if steps == 0:
            return # No movement needed

        # 5. Calculate increment per step
        increment = distance / steps

        self.get_logger().info(f"Moving {joint_type.upper()} to {target_angle_deg}° at {rpm} RPM ({duration:.2f}s)")

        # --- EXECUTION LOOP ---
        current_temp = start_angle
        msg = Float64()

        for _ in range(steps):
            current_temp += increment
            
            # Publish
            msg.data = math.radians(current_temp)
            publisher.publish(msg)
            
            # Update internal tracker
            if joint_type == 'pitch':
                self.current_pitch = current_temp
            else:
                self.current_yaw = current_temp
            
            # Sleep to maintain timing
            time.sleep(self.dt)

        # Ensure we land exactly on the target at the end
        msg.data = math.radians(target_angle_deg)
        publisher.publish(msg)
        
        if joint_type == 'pitch':
            self.current_pitch = target_angle_deg
        else:
            self.current_yaw = target_angle_deg


def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()

    try:
        # --- SEQUENCE START ---
        
        # 1. Move Pitch to -45 degrees
        controller.move_smoothly(joint_type='pitch', target_angle_deg=-45.0, rpm=2.0)
        time.sleep(0.5) 

        # 2. Move Pitch to 90 degrees at 10 RPM
        controller.move_smoothly(joint_type='pitch', target_angle_deg=90.0, rpm=2.0)
        time.sleep(0.5)

        # 3. Move Yaw (Base) to 90 degrees Left at 5 RPM
        controller.move_smoothly(joint_type='yaw', target_angle_deg=180.0, rpm=5.0)
        time.sleep(0.5)

        # 4. Repeat Pitch -45 at 10 RPM
        controller.move_smoothly(joint_type='pitch', target_angle_deg=-45.0, rpm=2.0)
        time.sleep(0.5)

        # 5. Repeat Pitch 90 at 10 RPM
        #controller.move_smoothly(joint_type='pitch', target_angle_deg=90.0, rpm=2.0)
        #time.sleep(0.5)

        controller.get_logger().info("Sequence Complete.")

    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
