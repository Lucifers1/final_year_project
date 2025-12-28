#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan, PointCloud2, PointField
from cv_bridge import CvBridge
import cv2
import numpy as np
import struct

class ColorScanNode(Node):
    def __init__(self):
        super().__init__('color_scan_node')
        
        self.bridge = CvBridge()
        self.current_rgb = (255, 255, 255)  # Default white

        # Subscriptions
        self.image_sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.image_callback, 10)
        
        # This is the line that was failing - it now points to the method below
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # Publisher
        self.pc_pub = self.create_publisher(PointCloud2, '/colored_scan', 10)
        self.get_logger().info("Color Scan Node started. Publishing to /colored_scan")

    def image_callback(self, msg):
        try:
            # Convert compressed image to OpenCV
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                return

            # Get dimensions and extract center column
            height, width, _ = cv_image.shape
            center_col = cv_image[:, width // 2]
            
            # Calculate average BGR and convert to RGB
            avg_bgr = np.mean(center_col, axis=0)
            self.current_rgb = (int(avg_bgr[2]), int(avg_bgr[1]), int(avg_bgr[0]))
            
        except Exception as e:
            self.get_logger().error(f'Image processing failed: {e}')

    def scan_callback(self, scan_msg):
        valid_points = []
        
        # Pack RGB into a single integer (Red, Green, Blue, Alpha)
        r, g, b = self.current_rgb
        # Pack as 4 bytes into one 32-bit unsigned int
        rgb_packed = struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]

        for i, r_val in enumerate(scan_msg.ranges):
            # Filter out invalid readings (inf, nan, or out of range)
            if np.isinf(r_val) or np.isnan(r_val) or r_val < scan_msg.range_min or r_val > scan_msg.range_max:
                continue
            
            # Calculate Cartesian coordinates
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            x = r_val * np.cos(angle)
            y = r_val * np.sin(angle)
            z = 0.0
            
            # Append binary data: 3 floats (x,y,z) + 1 uint32 (rgb) = 16 bytes
            valid_points.append(struct.pack('fffI', x, y, z, rgb_packed))

        if not valid_points:
            return

        # Create PointCloud2 message
        pc_msg = PointCloud2()
        pc_msg.header = scan_msg.header
        pc_msg.height = 1
        pc_msg.width = len(valid_points)
        pc_msg.is_dense = False
        pc_msg.is_bigendian = False

        # Define fields: x, y, z are FLOAT32 (7), rgb is UINT32 (6)
        pc_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        
        pc_msg.point_step = 16
        pc_msg.row_step = pc_msg.point_step * pc_msg.width
        pc_msg.data = b''.join(valid_points)

        self.pc_pub.publish(pc_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ColorScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
