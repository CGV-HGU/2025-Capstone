#!/usr/bin/env python3
import math
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, Header
from tf2_ros import TransformBroadcaster


class FakeLidarWithTF(Node):
    def __init__(self):
        super().__init__('fake_lidar_with_tf')

        # Configurable TF & relay parameters (lidar_x_offset는 실측 후 조정 가능)
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame', 'lidar_link')
        self.declare_parameter('lidar_x_offset', -0.55)
        self.declare_parameter('lidar_y_offset', 0.0)
        self.declare_parameter('lidar_z_offset', 0.0)
        self.declare_parameter('enable_scan_relay', False)
        self.declare_parameter('stale_timeout_sec', 0.5)

        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value
        self.lidar_x_offset = self.get_parameter('lidar_x_offset').get_parameter_value().double_value
        self.lidar_y_offset = self.get_parameter('lidar_y_offset').get_parameter_value().double_value
        self.lidar_z_offset = self.get_parameter('lidar_z_offset').get_parameter_value().double_value
        self.enable_scan_relay = self.get_parameter('enable_scan_relay').get_parameter_value().bool_value
        self.stale_timeout_sec = self.get_parameter('stale_timeout_sec').get_parameter_value().double_value

        # scan geometry parameters
        self.num_readings = 141
        self.angle_min = math.radians(-35.0)
        self.angle_max = math.radians(35.0)
        self.angle_increment = math.radians(0.5)
        self.range_min = 0.1
        self.range_max = 10.0

        # state
        self.floor_state = True  # True → no obstacle
        self.channel_distances = np.full(self.num_readings, float('inf'), dtype=float)
        self.last_channel_rx_wall = None
        self.upstream_stamp = None
        self.has_fresh_channels = False
        self.stale_warned = False

        # publishers & TF
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10) if self.enable_scan_relay else None
        self.tf_broadcaster = TransformBroadcaster(self)

        # subscribers
        self.create_subscription(
            Bool,
            '/floor_detector',
            self.floor_callback,
            10
        )
        self.create_subscription(
            Float32MultiArray,
            '/lidar_channel_distances',
            self.channel_callback,
            10
        )

        # timer @ 10Hz (TF broadcast; scan relay only when enabled & non-stale)
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            f'FakeLidarWithTF initialized (TF {self.parent_frame}->{self.child_frame} '
            f'xyz=({self.lidar_x_offset:.3f}, {self.lidar_y_offset:.3f}, {self.lidar_z_offset:.3f}), '
            f'scan_relay={self.enable_scan_relay}, stale_timeout={self.stale_timeout_sec:.2f}s).'
        )

    def floor_callback(self, msg: Bool):
        self.floor_state = msg.data

    @staticmethod
    def _parse_upstream_stamp(msg: Float32MultiArray):
        if msg.layout.dim:
            label = msg.layout.dim[0].label or ''
            if label.startswith('channels@'):
                try:
                    stamp_ns = int(label.split('@', 1)[1])
                    if stamp_ns > 0:
                        return RosTime(
                            sec=stamp_ns // 1_000_000_000,
                            nanosec=stamp_ns % 1_000_000_000
                        )
                except ValueError:
                    pass
        return None

    def channel_callback(self, msg: Float32MultiArray):
        data = np.array(msg.data, dtype=float)
        if data.size != self.num_readings:
            self.get_logger().warn(
                f'Expected {self.num_readings} channels, got {data.size}'
            )
            return
        self.channel_distances = data
        self.last_channel_rx_wall = time.monotonic()
        self.upstream_stamp = self._parse_upstream_stamp(msg) or self.get_clock().now().to_msg()
        self.has_fresh_channels = True
        self.stale_warned = False

    def timer_callback(self):
        now_ros = self.get_clock().now().to_msg()

        # 1) Static geometry TF broadcast
        t = TransformStamped()
        t.header.stamp = now_ros
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = float(self.lidar_x_offset)
        t.transform.translation.y = float(self.lidar_y_offset)
        t.transform.translation.z = float(self.lidar_z_offset)
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # 2) Optional LaserScan relay (never republishes expired or duplicate stale frames)
        if not self.enable_scan_relay or self.scan_pub is None:
            return
        if self.last_channel_rx_wall is None:
            return

        age_sec = time.monotonic() - self.last_channel_rx_wall
        if age_sec > self.stale_timeout_sec:
            if not self.stale_warned:
                self.get_logger().warn(
                    f'Upstream /lidar_channel_distances expired ({age_sec:.2f}s > {self.stale_timeout_sec:.2f}s); '
                    'halting /scan republish.'
                )
                self.stale_warned = True
            self.has_fresh_channels = False
            return

        if not self.has_fresh_channels:
            return

        scan = LaserScan()
        scan.header = Header(stamp=self.upstream_stamp or now_ros, frame_id=self.child_frame)
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.scan_time = 1.0 / 10.0
        scan.time_increment = 0.0
        scan.ranges = self.channel_distances[::-1].tolist()
        scan.intensities = [0.0] * self.num_readings
        self.scan_pub.publish(scan)
        self.has_fresh_channels = False

    def destroy_node(self):
        super().destroy_node()
        self.get_logger().info("Node destroyed.")


def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarWithTF()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
