#!/usr/bin/env python3
"""
ros1_serial_motor_node.py

ROS1 Noetic node to communicate with the Arduino PGM36-36ZY oscillation sketch.

Features
- Opens a serial port to the Arduino (default /dev/ttyUSB0, 9600 baud)
- Interactive console: enter PWM (0-255) and number of cycles (positive int)
- ROS topic subscriber `/motor/set_params` (std_msgs/Int32MultiArray) to set [pwm, cycles]
- Publishes raw Arduino lines on `/motor/telemetry_raw` (std_msgs/String)
- Publishes parsed telemetry on `/motor/telemetry` (std_msgs/Float32MultiArray)
  as [pps, pwm, ppm, rpm, angle_rad]

Usage
1) Install dependencies: `pip3 install pyserial`
2) Start roscore: `roscore`
3) Run this node:
   `python3 ros1_serial_motor_node.py _serial_port:=/dev/ttyUSB0 _baud:=9600`

Publishing params programmatically (instead of interactive console):
  rostopic pub /motor/set_params std_msgs/Int32MultiArray "data: [150, 3]" -1
  (will send PWM=150 and cycles=3)

Notes
- The script sends integers as newline-terminated lines (e.g. "150\n"), matching
  the Arduino's use of Serial.parseInt().
- If the Arduino resets when the serial port opens, the node waits 2 seconds
  before sending data to allow it to boot.

"""

import rospy
from std_msgs.msg import String, Float32MultiArray, Int32MultiArray
import serial
import threading
import time
import re
import sys


class SerialMotorNode(object):
    def __init__(self):
        rospy.init_node('serial_motor_node', anonymous=False)

        # ROS params
        self.serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
        self.baud = int(rospy.get_param('~baud', 9600))
        self.read_timeout = float(rospy.get_param('~read_timeout', 1.0))

        rospy.loginfo('Opening serial port: %s @ %d', self.serial_port, self.baud)

        # Publishers
        self.raw_pub = rospy.Publisher('motor/telemetry_raw', String, queue_size=10)
        self.parsed_pub = rospy.Publisher('motor/telemetry', Float32MultiArray, queue_size=10)

        # Subscriber to accept programmatic commands [pwm, cycles]
        self.sub = rospy.Subscriber('motor/set_params', Int32MultiArray, self._set_params_cb)

        # Serial
        try:
            self.ser = serial.Serial(self.serial_port, self.baud, timeout=self.read_timeout)
            time.sleep(2.0)  # allow Arduino to reset and boot
            rospy.loginfo('Serial port opened')
        except serial.SerialException as e:
            rospy.logerr('Failed to open serial port: %s', e)
            raise

        # Regex for parsing telemetry (supports both sampling and event lines)
        self.re_pwm = re.compile(r'PWM\s*[:|]\s*(\d+)')
        self.re_pps = re.compile(r'PPS\s*[:|]\s*(\d+)')
        self.re_ppm = re.compile(r'PPM\s*[:|]\s*(\d+)')
        self.re_rpm = re.compile(r'RPM\s*[:|]\s*([0-9]+(?:\.[0-9]+)?)')
        self.re_angle = re.compile(r'Angle\(rad\)\s*[:|]\s*([0-9]+(?:\.[0-9]+)?)')

        # Thread control
        self._stop_event = threading.Event()

        # Start threads
        self.reader_thread = threading.Thread(target=self._serial_reader, name='serial_reader')
        self.reader_thread.daemon = True
        self.reader_thread.start()

        self.input_thread = threading.Thread(target=self._console_input_loop, name='console_input')
        self.input_thread.daemon = True
        self.input_thread.start()

    def _serial_reader(self):
        rospy.logdebug('Serial reader thread started')
        while not rospy.is_shutdown() and not self._stop_event.is_set():
            try:
                raw = self.ser.readline()
            except Exception as e:
                rospy.logerr('Error reading serial: %s', e)
                break

            if not raw:
                continue

            try:
                line = raw.decode('utf-8', errors='replace').strip()
            except Exception:
                line = str(raw)

            if not line:
                continue

            # Publish raw
            msg = String()
            msg.data = line
            self.raw_pub.publish(msg)

            # Also print to ROS log (info level)
            rospy.loginfo('%s', line)

            # Parse telemetry fields
            try:
                pwm_m = self.re_pwm.search(line)
                pps_m = self.re_pps.search(line)
                ppm_m = self.re_ppm.search(line)
                rpm_m = self.re_rpm.search(line)
                ang_m = self.re_angle.search(line)

                if pwm_m or pps_m or ppm_m or rpm_m or ang_m:
                    pps = float(pps_m.group(1)) if pps_m else 0.0
                    pwm = float(pwm_m.group(1)) if pwm_m else 0.0
                    ppm = float(ppm_m.group(1)) if ppm_m else 0.0
                    rpm = float(rpm_m.group(1)) if rpm_m else 0.0
                    ang = float(ang_m.group(1)) if ang_m else 0.0

                    arr = Float32MultiArray()
                    arr.data = [pps, pwm, ppm, rpm, ang]
                    self.parsed_pub.publish(arr)
            except Exception as e:
                rospy.logwarn('Failed to parse telemetry: %s', e)

        rospy.loginfo('Serial reader thread exiting')

    def _console_input_loop(self):
        rospy.loginfo('\n=== Interactive console input started ===')
        while not rospy.is_shutdown() and not self._stop_event.is_set():
            try:
                inp = raw_input('\nEnter PWM value (0-255) or q to quit: ') if sys.version_info[0] < 3 else input('\nEnter PWM value (0-255) or q to quit: ')
            except EOFError:
                rospy.loginfo('Input closed (EOF). Stopping input loop.')
                self._stop_event.set()
                break

            if inp is None:
                continue
            s = inp.strip()
            if s == '':
                continue
            if s.lower() == 'q':
                rospy.loginfo('Quit requested by user')
                self._stop_event.set()
                break

            # Parse PWM
            try:
                pwm = int(s)
                if not (0 <= pwm <= 255):
                    print('PWM must be between 0 and 255')
                    continue
            except ValueError:
                print('Please enter an integer PWM value (0-255)')
                continue

            # Send PWM
            try:
                self.ser.write((str(pwm) + '\n').encode('utf-8'))
                rospy.loginfo('Sent PWM: %d', pwm)
            except Exception as e:
                rospy.logerr('Failed to write PWM to serial: %s', e)
                self._stop_event.set()
                break

            # Ask for cycles
            try:
                inp2 = raw_input('Enter number of cycles (positive integer) or q to quit: ') if sys.version_info[0] < 3 else input('Enter number of cycles (positive integer) or q to quit: ')
            except EOFError:
                rospy.loginfo('Input closed (EOF). Stopping input loop.')
                self._stop_event.set()
                break

            if inp2 is None:
                continue
            s2 = inp2.strip()
            if s2 == '':
                continue
            if s2.lower() == 'q':
                rospy.loginfo('Quit requested by user')
                self._stop_event.set()
                break

            try:
                cycles = int(s2)
                if cycles <= 0:
                    print('Cycles must be a positive integer')
                    continue
            except ValueError:
                print('Please enter an integer for cycles')
                continue

            # Send cycles
            try:
                self.ser.write((str(cycles) + '\n').encode('utf-8'))
                rospy.loginfo('Sent cycles: %d', cycles)
            except Exception as e:
                rospy.logerr('Failed to write cycles to serial: %s', e)
                self._stop_event.set()
                break

        rospy.loginfo('Input thread exiting')

    def _set_params_cb(self, msg):
        # msg is Int32MultiArray; expect at least two elements: [pwm, cycles]
        try:
            data = list(msg.data)
            if len(data) < 2:
                rospy.logwarn('motor/set_params requires at least two integers [pwm, cycles]')
                return
            pwm = int(data[0])
            cycles = int(data[1])
            if not (0 <= pwm <= 255):
                rospy.logwarn('PWM out of range: %d', pwm)
                return
            if cycles <= 0:
                rospy.logwarn('Cycles must be positive: %d', cycles)
                return

            # send pwm and cycles as newline-terminated integers
            self.ser.write((str(pwm) + '\n').encode('utf-8'))
            time.sleep(0.05)
            self.ser.write((str(cycles) + '\n').encode('utf-8'))
            rospy.loginfo('Sent parameters from topic: PWM=%d Cycles=%d', pwm, cycles)
        except Exception as e:
            rospy.logerr('Failed to handle set_params message: %s', e)

    def shutdown(self):
        rospy.loginfo('Shutting down SerialMotorNode...')
        self._stop_event.set()
        try:
            if self.reader_thread.is_alive():
                self.reader_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self.input_thread.is_alive():
                # no portable way to interrupt input(), so best-effort
                pass
        except Exception:
            pass
        try:
            if hasattr(self, 'ser') and self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass


def main():
    try:
        node = SerialMotorNode()
    except Exception as e:
        rospy.logerr('Failed to start SerialMotorNode: %s', e)
        return

    # Keep alive until shutdown
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('KeyboardInterrupt received')
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()

