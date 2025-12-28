#!/usr/bin/env python3

import rospy
import numpy as np
import math
import cv2
from sensor_msgs.msg import LaserScan, Image, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import struct
from tf.transformations import euler_matrix, translation_matrix

class LidarCameraVisualizer:
    def __init__(self):
        rospy.init_node("lidar_camera_colored")

        self.bridge = CvBridge()
        self.latest_image = None

        # Subscribers
        rospy.Subscriber("/image_raw", Image, self.image_cb, queue_size=1)
        rospy.Subscriber("/scan", LaserScan, self.lidar_cb, queue_size=1)

        # Publisher
        self.pub = rospy.Publisher("/colored_pointcloud", PointCloud2, queue_size=1)

        # Transformation: LIDAR → Camera
        # Assuming camera and LIDAR are aligned at origin. Change if mounted differently.
        self.T_lidar_cam = np.eye(4)  # 4x4 homogeneous transform

        rospy.loginfo("Lidar-Camera colored visualizer running")

    def image_cb(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"CV Bridge error: {e}")

    def lidar_cb(self, scan):
        if self.latest_image is None:
            return

        img_h, img_w, _ = self.latest_image.shape
        points = []
        angle = scan.angle_min

        for r in scan.ranges:
            if np.isfinite(r):
                # LIDAR coordinates (x, y, z)
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0
                lidar_point = np.array([x, y, z, 1.0])

                # Transform to camera frame
                cam_point = self.T_lidar_cam @ lidar_point
                cx, cy, cz = cam_point[:3]

                # Simple pinhole projection
                if cz <= 0:
                    angle += scan.angle_increment
                    continue

                fx = fy = 500  # focal length in pixels (adjust)
                cx_img = img_w / 2
                cy_img = img_h / 2

                u = int((cx * fx) / cz + cx_img)
                v = int((cy * fy) / cz + cy_img)

                if 0 <= u < img_w and 0 <= v < img_h:
                    b, g, r = self.latest_image[v, u]
                    rgb_uint32 = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
                    points.append([x, y, z, rgb_uint32])

            angle += scan.angle_increment

        # Create PointCloud2
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = scan.header.frame_id

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1)
        ]

        pc2_msg = pc2.create_cloud(header, fields, points)
        self.pub.publish(pc2_msg)

if __name__ == "__main__":
    try:
        LidarCameraVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
