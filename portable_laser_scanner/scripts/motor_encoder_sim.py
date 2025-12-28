#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
import math
import threading

class MotorEncoderSim(Node):
    def __init__(self):
        super().__init__('motor_encoder_sim')

        self.pub_joint = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_encoder = self.create_publisher(Int32, 'encoder_count', 10)

        self.joint_name = 'base_to_bar'
        self.timer_period = 0.02  # 50 Hz
        self.timer = self.create_timer(self.timer_period, self.update_motor)

        # Encoder specs
        self.pulses_per_rev = 5904  # exact pulses per full rotation
        self.encoder_count = 0      # integer in range [0, pulses_per_rev-1]
        self._pulse_accumulator = 0.0  # fractional pulse leftover accumulator

        # Motor state
        self.current_angle = 0.0    # radians
        self.current_rpm = 0.0

        # Start input thread
        threading.Thread(target=self.get_user_input, daemon=True).start()
        self.get_logger().info("Motor Encoder Simulator started. Enter desired RPM (0 to reset encoder).")

    def get_user_input(self):
        while rclpy.ok():
            try:
                val = input("Enter RPM (positive = forward, negative = reverse, 0 = stop & reset): ").strip()
                if val == '':
                    continue
                rpm = float(val)
                # If rpm == 0, reset encoder count to 0 immediately (as requested)
                if abs(rpm) < 1e-9:
                    self.current_rpm = 0.0
                    self.current_angle = 0.0
                    self._pulse_accumulator = 0.0
                    self.encoder_count = 0
                    self.get_logger().info("RPM set to 0 - encoder count reset to 0")
                else:
                    self.current_rpm = rpm
                    self.get_logger().info(f"Set RPM: {rpm}")
            except ValueError:
                print("Invalid input. Enter a number (e.g. 21, -10, 0)")

    def update_motor(self):
        # Compute angular velocity (rad/s) from RPM
        angular_velocity = (self.current_rpm * 2.0 * math.pi) / 60.0

        # Integrate angle
        delta_angle = angular_velocity * self.timer_period
        self.current_angle += delta_angle
        # Keep angle in [0, 2pi) for predictability
        self.current_angle = self.current_angle % (2.0 * math.pi)

        # Compute pulses per radian
        pulses_per_radian = self.pulses_per_rev / (2.0 * math.pi)

        # fractional pulses for this timestep:
        delta_pulses_f = delta_angle * pulses_per_radian

        # accumulate fractional pulses to avoid lost fractions
        self._pulse_accumulator += delta_pulses_f

        # extract integer pulses to apply now (can be negative)
        int_pulses = int(math.floor(self._pulse_accumulator)) if self._pulse_accumulator >= 0 else int(math.ceil(self._pulse_accumulator))

        # subtract applied integer pulses from accumulator (keep fractional remainder)
        self._pulse_accumulator -= int_pulses

        # update encoder_count with wrapping behavior
        if int_pulses != 0:
            # positive or negative wrap-around
            self.encoder_count = (self.encoder_count + int_pulses) % self.pulses_per_rev

        # Publish JointState for RViz visualization
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [self.joint_name]
        joint_msg.position = [self.current_angle]
        self.pub_joint.publish(joint_msg)

        # Publish encoder count (Int32)
        enc_msg = Int32()
        enc_msg.data = int(self.encoder_count)
        self.pub_encoder.publish(enc_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorEncoderSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

