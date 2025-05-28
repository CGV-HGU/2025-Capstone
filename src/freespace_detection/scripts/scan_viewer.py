#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class ScanViewer(Node):
    def __init__(self):
        super().__init__('scan_viewer')
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.get_logger().info('ScanViewer initialized, waiting for /scan...')

    def scan_callback(self, msg: LaserScan):
        # ranges를 numpy array로 변환
        ranges = np.array(msg.ranges, dtype=float)

        # 전체 값을 한 번에 출력 (길어서 불편하면 슬라이스로 잘라 출력하세요)
        self.get_logger().info(f'Received {ranges.size} ranges:')
        self.get_logger().info(str(ranges.tolist()))

        # 통계 출력
        valid = ranges[np.isfinite(ranges)]
        if valid.size > 0:
            mn, mx, avg = valid.min(), valid.max(), valid.mean()
            self.get_logger().info(
                f'  valid min={mn:.2f}, max={mx:.2f}, mean={avg:.2f}'
            )
        else:
            self.get_logger().info('  all values are inf (no valid readings)')

def main(args=None):
    rclpy.init(args=args)
    node = ScanViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
