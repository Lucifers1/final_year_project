#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
import threading

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        port = '/dev/ttyUSB0'   # Arduino port
        baud = 9600

        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2)  # wait for Arduino to reset
            self.get_logger().info(f'✅ Connected to Arduino on {port}')
        except Exception as e:
            self.get_logger().error(f'❌ Cannot open serial port: {e}')
            raise SystemExit

        # Start background thread to read Arduino feedback continuously
        self.keep_reading = True
        threading.Thread(target=self.read_serial, daemon=True).start()

        self.get_logger().info("Ready! Enter motor commands below:")
        self.get_logger().info("Format: PWM (0–255), Direction (R/L), Angle (degrees)\n")

        self.user_input_loop()

    # === Thread: Read Arduino messages ===
    def read_serial(self):
        while self.keep_reading:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode(errors='ignore').strip()
                    if line:
                        self.get_logger().info(f'[Arduino] {line}')
                time.sleep(0.05)
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
                self.keep_reading = False

    # === Main user input loop ===
    def user_input_loop(self):
        while rclpy.ok():
            try:
                pwm = input("\nEnter PWM (0–255): ").strip()
                if not pwm.isdigit():
                    self.get_logger().warn("⚠️ Invalid PWM value. Try again.")
                    continue

                direction = input("Enter direction (R/L): ").strip().upper()
                if direction not in ['R', 'L']:
                    self.get_logger().warn("⚠️ Invalid direction. Use R or L.")
                    continue

                angle_str = input("Enter target angle (degrees): ").strip()
                try:
                    angle = float(angle_str)
                    if angle <= 0:
                        raise ValueError
                except ValueError:
                    self.get_logger().warn("⚠️ Invalid angle. Must be positive number.")
                    continue

                # Send commands step by step
                self.send_command(pwm, direction, angle)

            except KeyboardInterrupt:
                self.get_logger().info("Shutting down...")
                self.keep_reading = False
                break

    # === Send commands to Arduino ===
    def send_command(self, pwm, direction, angle):
        try:
            self.ser.reset_input_buffer()
            self.ser.write(f"{pwm}\n".encode())
            time.sleep(1)

            self.ser.write(f"{direction}\n".encode())
            time.sleep(1)

            self.ser.write(f"{angle}\n".encode())
            time.sleep(1)

            self.get_logger().info(f"✅ Command sent: PWM={pwm}, Dir={direction}, Angle={angle}")

        except Exception as e:
            self.get_logger().error(f'❌ Error sending command: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.keep_reading = False
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
