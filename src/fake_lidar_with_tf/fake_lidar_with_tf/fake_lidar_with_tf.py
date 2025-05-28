#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Header
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import numpy as np

class FakeLidarWithTF(Node):
    def __init__(self):
        super().__init__('fake_lidar_with_tf')

        # parameters
        self.num_readings = 141
        # 카메라 수평 FOV: -35° ~ +35°
        self.angle_min = math.radians(-35.0)
        self.angle_max = math.radians( 35.0)
        # 각도 간격 0.5°
        self.angle_increment = math.radians(0.5)
        self.range_min = 0.1
        self.range_max = 10.0

        # state
        self.floor_state = True  # True → no obstacle
        self.channel_distances = np.full(self.num_readings, float('inf'), dtype=float)

        # publishers & TF
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # subscribers
        self.create_subscription(Bool,
                                 '/floor_detector',
                                 self.floor_callback,
                                 10)
        self.create_subscription(Float32MultiArray,
                                 '/lidar_channel_distances',
                                 self.channel_callback,
                                 10)

        # timer @ 10Hz
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('FakeLidarWithTF initialized with FOV ±35° @ 0.5° step.')

    def floor_callback(self, msg: Bool):
        # False → 장애물 있음
        self.floor_state = msg.data

    def channel_callback(self, msg: Float32MultiArray):
        data = np.array(msg.data, dtype=float)
        if data.size != self.num_readings:
            self.get_logger().warn(
                f'Expected {self.num_readings} channels, got {data.size}')
            return
        self.channel_distances = data

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        # 1) TF broadcast
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'lidar_link'
        t.transform.translation.x = -0.55
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # 2) LaserScan 메시지 생성
        scan = LaserScan()
        scan.header = Header(stamp=now, frame_id='lidar_link')
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.scan_time       = 1.0 / 10.0      # <- 추가
        scan.time_increment  = scan.scan_time / self.num_readings  # <- 추가

        if not self.floor_state:
            # 장애물 있을 때: LUT 기반 거리 사용
            scan.ranges = self.channel_distances[::-1].tolist()
        else:
            # 장애물 없을 때: 모두 inf
            scan.ranges = [float('inf')] * self.num_readings

        scan.intensities = [0.0] * self.num_readings

        # 3) 퍼블리시
        self.scan_pub.publish(scan)

    def destroy_node(self):
        super().destroy_node()
        self.get_logger().info("Node destroyed.")

def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarWithTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
