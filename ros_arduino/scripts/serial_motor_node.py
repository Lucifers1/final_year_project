#!/usr/bin/env python3
"""
ros2_serial_motor_node.py

ROS2 Humble node (Python) to communicate with an Arduino over serial for the
PGM36-36ZY oscillation motor sketch. It mirrors the Arduino serial interaction
from your sketch: it sends two integers (PWM and cycles) to the Arduino and
reads the textual telemetry printed by the Arduino (lines that include PWM,
PPS, PPM, RPM). Telemetry is published on a ROS2 topic and printed to console.

Requirements
- ROS2 Humble (rclpy available in environment)
- pyserial: pip install pyserial

Usage
1) Source ROS2 environment (and your workspace if needed):
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash   # if you want

2) Run the node (default serial port /dev/ttyUSB0, default baud 9600):
   python3 ros2_serial_motor_node.py --ros-args -p serial_port:="/dev/ttyUSB0" -p baudrate:=9600

3) Follow the interactive prompts in the terminal. Enter PWM (0-255) and then
   number of cycles. The script sends each integer line to the Arduino and
   then listens for telemetry lines from the Arduino and publishes them to
   ROS2 topic `/motor/telemetry_raw` (std_msgs/String) and parsed numeric
   values on `/motor/telemetry` (std_msgs/Float32MultiArray) as [pps, pwm, ppm, rpm].

Notes
- The Arduino sketch expects integer values via Serial.parseInt(), so we send
  each integer followed by a newline.
- The node uses two background threads: one for reading serial, one for reading
  user input. The ROS spin happens in the main thread.

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import serial
import threading
import time
import re
import sys


class SerialMotorNode(Node):
    def __init__(self):
        super().__init__('serial_motor_node')

        # ROS parameters (can be overridden via --ros-args -p ...)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('read_timeout', 1.0)

        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = int(self.get_parameter('baudrate').get_parameter_value().integer_value)
        self.read_timeout = float(self.get_parameter('read_timeout').get_parameter_value().double_value)

        self.get_logger().info(f'Opening serial port: {self.serial_port} @ {self.baudrate} baud')

        # Publishers
        self.raw_pub = self.create_publisher(String, 'motor/telemetry_raw', 10)
        self.parsed_pub = self.create_publisher(Float32MultiArray, 'motor/telemetry', 10)

        # Serial port
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=self.read_timeout)
            # Small delay to allow Arduino to reset (if it does on open)
            time.sleep(2.0)
            self.get_logger().info('Serial port opened successfully')
        except serial.SerialException as e:
            self.get_logger().error(f'Could not open serial port: {e}')
            raise

        # Thread control
        self._stop_event = threading.Event()

        # Start background threads
        self.reader_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self.reader_thread.start()

        self.input_thread = threading.Thread(target=self._console_input_loop, daemon=True)
        self.input_thread.start()

        # Regex to parse telemetry lines like: "PWM: 150   PPS: 123   PPM: 7380   RPM: 1.25"
        self.re_pwm = re.compile(r'PWM\s*:\s*(\d+)')
        self.re_pps = re.compile(r'PPS\s*:\s*(\d+)')
        self.re_ppm = re.compile(r'PPM\s*:\s*(\d+)')
        self.re_rpm = re.compile(r'RPM\s*:\s*([0-9]+(?:\.[0-9]+)?)')

    def _serial_reader(self):
        """Continuously read lines from serial, publish raw and parsed telemetry."""
        self.get_logger().debug('Serial reader thread started')
        while not self._stop_event.is_set() and rclpy.ok():
            try:
                raw = self.ser.readline()
            except Exception as e:
                self.get_logger().error(f'Error reading serial: {e}')
                break

            if not raw:
                # timeout or empty, continue
                continue

            try:
                line = raw.decode('utf-8', errors='replace').strip()
            except Exception:
                line = str(raw)

            if not line:
                continue

            # Publish raw string
            msg = String()
            msg.data = line
            self.raw_pub.publish(msg)

            # Print to console
            print(line)

            # Try to parse numeric telemetry
            try:
                pwm_m = self.re_pwm.search(line)
                pps_m = self.re_pps.search(line)
                ppm_m = self.re_ppm.search(line)
                rpm_m = self.re_rpm.search(line)

                if pps_m or pwm_m or ppm_m or rpm_m:
                    pps = float(pps_m.group(1)) if pps_m else 0.0
                    pwm = float(pwm_m.group(1)) if pwm_m else 0.0
                    ppm = float(ppm_m.group(1)) if ppm_m else 0.0
                    rpm = float(rpm_m.group(1)) if rpm_m else 0.0

                    arr = Float32MultiArray()
                    # We'll publish [pps, pwm, ppm, rpm]
                    arr.data = [pps, pwm, ppm, rpm]
                    self.parsed_pub.publish(arr)
            except Exception as e:
                self.get_logger().warn(f'Failed parsing telemetry: {e}')

        self.get_logger().info('Serial reader thread exiting')

    def _console_input_loop(self):
        """Interactively take user input (PWM and cycles) and send integers to Arduino.

        This mirrors the Arduino sketch behavior: first enter PWM value, then number
        of cycles. Each integer is sent as a line (\n-terminated), which the
        Arduino's Serial.parseInt() will read.
        """
        self.get_logger().info('\n=== Interactive console input started ===')
        try:
            while not self._stop_event.is_set() and rclpy.ok():
                # Ask for PWM
                pwm = None
                while pwm is None and rclpy.ok() and not self._stop_event.is_set():
                    try:
                        inp = input('\nEnter PWM value (0-255) or q to quit: ').strip()
                    except EOFError:
                        # If input closed, stop
                        self.get_logger().info('Input closed (EOF). Stopping input loop.')
                        self._stop_event.set()
                        break

                    if inp.lower() == 'q':
                        self.get_logger().info('Quit requested by user')
                        self._stop_event.set()
                        break

                    if inp == '':
                        continue

                    try:
                        v = int(inp)
                        if 0 <= v <= 255:
                            pwm = v
                        else:
                            print('PWM must be between 0 and 255')
                    except ValueError:
                        print('Please enter an integer PWM value (0-255)')

                if self._stop_event.is_set():
                    break

                # Send PWM
                line = f"{pwm}\n"
                try:
                    self.ser.write(line.encode('utf-8'))
                    self.get_logger().info(f'Sent PWM: {pwm}')
                except Exception as e:
                    self.get_logger().error(f'Failed to write PWM to serial: {e}')
                    break

                # Ask for cycles
                cycles = None
                while cycles is None and rclpy.ok() and not self._stop_event.is_set():
                    try:
                        inp = input('Enter number of cycles (positive integer) or q to quit: ').strip()
                    except EOFError:
                        self.get_logger().info('Input closed (EOF). Stopping input loop.')
                        self._stop_event.set()
                        break

                    if inp.lower() == 'q':
                        self.get_logger().info('Quit requested by user')
                        self._stop_event.set()
                        break

                    if inp == '':
                        continue

                    try:
                        c = int(inp)
                        if c > 0:
                            cycles = c
                        else:
                            print('Cycles must be a positive integer')
                    except ValueError:
                        print('Please enter an integer for cycles')

                if self._stop_event.is_set():
                    break

                # Send cycles
                line = f"{cycles}\n"
                try:
                    self.ser.write(line.encode('utf-8'))
                    self.get_logger().info(f'Sent cycles: {cycles}\n')
                except Exception as e:
                    self.get_logger().error(f'Failed to write cycles to serial: {e}')
                    break

                # After sending the pair, the Arduino should run and print telemetry
                # which will be captured by the reader thread. Loop and prompt again
                # when done (or allow Arduino/arduino-sketch to prompt for new input).

            self.get_logger().info('Input thread exiting')
        except Exception as e:
            self.get_logger().error(f'Exception in input thread: {e}')
            self._stop_event.set()

    def destroy_node(self):
        # Stop threads and close serial
        self.get_logger().info('Shutting down SerialMotorNode...')
        self._stop_event.set()
        try:
            if hasattr(self, 'reader_thread') and self.reader_thread.is_alive():
                self.reader_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if hasattr(self, 'input_thread') and self.input_thread.is_alive():
                # input() may be blocking; best-effort to close
                # There's no safe portable way to interrupt input(); rely on user to press Enter or send EOF
                pass
        except Exception:
            pass

        try:
            if hasattr(self, 'ser') and self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # Create node
    try:
        node = SerialMotorNode()
    except Exception as e:
        print(f'Failed to start node: {e}', file=sys.stderr)
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, shutting down')
    finally:
        # Clean shutdown
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()

