import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, Bool
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math


class FakeLidarWithTF(Node):
    def __init__(self):
        super().__init__('fake_lidar_with_tf')

        # 퍼블리셔와 TF 브로드캐스터 생성
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # /floor_detector 토픽 구독자 생성
        self.floor_subscriber = self.create_subscription(
            Bool,
            '/floor_detector',
            self.floor_callback,
            10
        )
        # 기본 floor state 값 (False이면 현재 라이다 동작 그대로, True이면 장애물 없음)
        self.floor_state = True

        # 라이다 메시지 발행 타이머 (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 라이다 센서 설정 값
        self.frame_id = 'lidar_link'
        self.parent_frame = 'base_link'
        self.angle_min = -math.pi/2.0
        self.angle_max = math.pi/2.0
        self.angle_increment = math.radians(0.5)
        self.range_min = 0.1
        self.range_max = 10.0  # 라이다 최대 감지거리

    def floor_callback(self, msg: Bool):
        # /floor_detector 토픽에서 수신한 값을 저장
        self.floor_state = msg.data

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        # TF 브로드캐스트: base_link -> lidar_link
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.frame_id
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        # LaserScan 메시지 생성
        scan = LaserScan()
        scan.header = Header()
        scan.header.stamp = now
        scan.header.frame_id = self.frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        num_readings = int((self.angle_max - self.angle_min) / self.angle_increment)
        # 기본: 모든 센서값을 최대 거리로 설정 (즉, 아무것도 감지되지 않음)
        # scan.ranges = [self.range_max] * num_readings
        scan.ranges = [float('inf')] * num_readings # gpt practice
        scan.intensities = [1.0] * num_readings

        # floor_state가 False일 때만 정면에 장애물이 있다고 가정하여 값을 변경
        if not self.floor_state:
            # 정면 각도 범위: -10° ~ +5°
            start_angle = math.radians(-12)
            end_angle = math.radians(12)
            start_idx = int((start_angle - self.angle_min) / self.angle_increment)
            end_idx = int((end_angle - self.angle_min) / self.angle_increment)
            for i in range(start_idx, end_idx + 1):
                scan.ranges[i] = 3.0 # 정면 장애물 거리 설정

        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarWithTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()