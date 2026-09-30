#!/usr/bin/env python3
"""
==============================================================================
ROS 2 Baseline Depth-to-LaserScan Perception Node
==============================================================================
This node replaces the V-LiDAR pipeline (floor_detector + fake_lidar_with_tf)
with an end-to-end Monocular Depth Estimation (MDE) baseline:
- Depth Anything V2 (Small) or MiDaS Small

Architecture & Dataflow:
[ camera/image_raw ] (USB Webcam)
        │
        ▼
[ MDE Inference Engine ] (Depth Anything V2 / MiDaS)
        │
        ▼ (Dense Depth Map: H x W)
[ Planar 2D LaserScan Slicing ] (Standard depthimage_to_laserscan geometry)
        │
        ├──► /scan (sensor_msgs/LaserScan, 141ch, ±35° FOV @ 10Hz)
        └──► TF Broadcaster: base_link -> lidar_link (x = -0.55m)
==============================================================================
"""

import os
import sys
import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

class BaselineDepthScanNode(Node):
    def __init__(self):
        super().__init__('baseline_depth_scan_node')

        # Declare ROS 2 parameters
        self.declare_parameter('model_type', 'depth_anything')  # 'depth_anything' or 'midas'
        self.declare_parameter('device', 'cpu')                 # 'cpu', 'cuda', or 'openvino'
        self.declare_parameter('target_rate', 10.0)             # Hz
        self.declare_parameter('input_width', 320)
        self.declare_parameter('input_height', 256)
        self.declare_parameter('num_channels', 141)

        self.model_type = self.get_parameter('model_type').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.target_rate = self.get_parameter('target_rate').get_parameter_value().double_value
        self.target_w = self.get_parameter('input_width').get_parameter_value().integer_value
        self.target_h = self.get_parameter('input_height').get_parameter_value().integer_value
        self.num_channels = self.get_parameter('num_channels').get_parameter_value().integer_value

        # LaserScan Specifications (matching fake_lidar_with_tf)
        self.angle_min = math.radians(-35.0)
        self.angle_max = math.radians(35.0)
        self.angle_increment = math.radians(0.5)
        self.range_min = 0.1
        self.range_max = 10.0

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publisher & Subscriber
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 1
        )

        self.get_logger().info(f"Initializing MDE Baseline Node [{self.model_type}] on device [{self.device}]...")
        self.init_model()

        self.last_infer_time = time.time()
        self.min_interval = 1.0 / self.target_rate
        self.get_logger().info("BaselineDepthScanNode ready and publishing to /scan.")

    def init_model(self):
        """Loads the selected deep MDE model."""
        import torch
        if self.model_type == 'midas':
            self.get_logger().info("Loading MiDaS_small via torch.hub...")
            self.model = torch.hub.load("intel-isl/MiDaS", "MiDaS_small", trust_repo=True)
            self.model.to(self.device)
            self.model.eval()
            midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms", trust_repo=True)
            self.transform = midas_transforms.small_transform
        else:
            self.get_logger().info("Loading Depth Anything V2 Small from HuggingFace...")
            try:
                from transformers import AutoImageProcessor, AutoModelForDepthEstimation
                model_id = "depth-anything/Depth-Anything-V2-Small-hf"
                self.processor = AutoImageProcessor.from_pretrained(model_id)
                self.model = AutoModelForDepthEstimation.from_pretrained(model_id).to(self.device)
                self.model.eval()
                self.is_hf = True
            except Exception as e:
                self.get_logger().warn(f"HuggingFace loader failed ({e}), falling back to torch.hub...")
                self.model = torch.hub.load("intel-isl/MiDaS", "MiDaS_small", trust_repo=True)
                self.model.to(self.device)
                self.model.eval()
                midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms", trust_repo=True)
                self.transform = midas_transforms.small_transform
                self.is_hf = False

    def image_callback(self, data):
        now = time.time()
        if now - self.last_infer_time < self.min_interval:
            return
        self.last_infer_time = now

        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        import torch
        img_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

        # 1. MDE Inference
        t0 = time.perf_counter()
        if getattr(self, 'is_hf', False):
            inputs = self.processor(images=img_rgb, return_tensors="pt").to(self.device)
            with torch.no_grad():
                outputs = self.model(**inputs)
                pred = outputs.predicted_depth
                pred_interpolated = torch.nn.functional.interpolate(
                    pred.unsqueeze(1),
                    size=(self.target_h, self.target_w),
                    mode="bicubic",
                    align_corners=False,
                ).squeeze()
                depth_map = pred_interpolated.cpu().numpy()
        else:
            input_batch = self.transform(img_rgb).to(self.device)
            with torch.no_grad():
                pred = self.model(input_batch)
                pred_interpolated = torch.nn.functional.interpolate(
                    pred.unsqueeze(1),
                    size=(self.target_h, self.target_w),
                    mode="bicubic",
                    align_corners=False,
                ).squeeze()
                depth_inv = pred_interpolated.cpu().numpy()
                depth_map = 1000.0 / (depth_inv + 1e-4)

        infer_time_ms = (time.perf_counter() - t0) * 1000.0

        # Normalize to metric scale (m)
        d_min, d_max = depth_map.min(), depth_map.max()
        if d_max > d_min:
            metric_depth = (depth_map - d_min) / (d_max - d_min) * 5.0 + 0.5
        else:
            metric_depth = depth_map

        # 2. Slice 2D scan from horizontal detection band (rows 120 ~ 200)
        h_slice = metric_depth[120:200, :]
        col_min = h_slice.min(axis=0)

        step = len(col_min) / float(self.num_channels)
        channel_dist = [float(col_min[int(i * step)]) for i in range(self.num_channels)]
        channel_dist = np.clip(channel_dist, self.range_min, self.range_max)

        # 3. Publish LaserScan & TF
        stamp = self.get_clock().now().to_msg()

        # TF Broadcast: base_link -> lidar_link
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'lidar_link'
        t.transform.translation.x = -0.55
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # LaserScan Message
        scan = LaserScan()
        scan.header = Header(stamp=stamp, frame_id='lidar_link')
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.scan_time = 1.0 / self.target_rate
        scan.time_increment = 0.0
        scan.ranges = channel_dist[::-1]  # Flip to match coordinate orientation
        scan.intensities = [0.0] * self.num_channels

        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = BaselineDepthScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
