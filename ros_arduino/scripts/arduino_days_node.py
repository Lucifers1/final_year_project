#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time

class ArduinoDays(Node):
    def __init__(self):
        super().__init__('arduino_days')
        # Change port if needed (e.g. '/dev/ttyUSB0' on some boards)
        port = '/dev/ttyUSB0'   # ✅ updated port
        baud = 9600

        try:
            self.ser = serial.Serial(port, baud, timeout=2)
        except Exception as e:
            self.get_logger().error(f"Cannot open serial port {port}: {e}")
            raise

        # allow Arduino to reset and print ready
        time.sleep(2)
        # flush any initial lines
        self.ser.reset_input_buffer()

        self.get_logger().info("Started arduino_days node (serial {})".format(port))
        # send once, then stop — change to timer if you want periodic sends
        self.send_and_receive("1,2,3,4,5,6,7")

    def send_and_receive(self, csv_numbers):
        # ensure newline
        msg = csv_numbers.strip() + "\n"
        self.get_logger().info(f"Sending: {csv_numbers}")
        self.ser.write(msg.encode('utf-8'))

        # read response line (Arduino ends with newline)
        try:
            line = self.ser.readline().decode('utf-8').strip()
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")
            line = ""

        if line:
            self.get_logger().info(f"Received from Arduino: {line}")
        else:
            self.get_logger().warn("No response or empty response from Arduino")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArduinoDays()
        # Keep node alive briefly so logs stay visible; shut down after a short delay
        # If you want continuous operation, use rclpy.spin(node) and move send_and_receive to a timer.
        time.sleep(0.5)
    finally:
        try:
            node.ser.close()
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
