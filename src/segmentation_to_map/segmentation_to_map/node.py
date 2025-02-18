#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_point

class SegmentationToMap(Node):
    def __init__(self):
        super().__init__('segmentation_to_map')

        # 세그멘테이션 이미지 Subscriber
        self.segmentation_sub = self.create_subscription(
            Image, '/camera/image_raw', self.segmentation_callback, 10
        )

        # 카메라 정보 Subscriber
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10
        )

        # TF Buffer와 Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()
        self.camera_info = None

        self.get_logger().info("Segmentation to Map Node Initialized")

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def segmentation_callback(self, msg):
        if self.camera_info is None:
            self.get_logger().warn("CameraInfo is not yet available.")
            return

        try:
            # ROS Image 메시지를 OpenCV 이미지로 변환
            segmentation_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # 특정 클래스 (바닥)만 추출
        mask = (segmentation_image == 1).astype(np.uint8)

        # 픽셀 좌표 추출
        pixels = np.column_stack(np.where(mask > 0))  # [[y1, x1], [y2, x2], ...]

        # 픽셀 좌표 -> 센서 좌표 변환
        fx, fy = self.camera_info.k[0], self.camera_info.k[4]
        cx, cy = self.camera_info.k[2], self.camera_info.k[5]
        sensor_coords = []

        for y, x in pixels:
            X = (x - cx) / fx
            Y = (y - cy) / fy
            Z = 1.0  # Depth 값 (임의 설정)
            sensor_coords.append((X, Y, Z))

        # 센서 좌표 -> 맵 좌표 변환
        map_coords = []
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", self.camera_info.header.frame_id, rclpy.time.Time()
            )
            for X, Y, Z in sensor_coords:
                point = PointStamped()
                point.header.frame_id = self.camera_info.header.frame_id
                point.point.x, point.point.y, point.point.z = X, Y, Z
                map_point = do_transform_point(point, transform)
                map_coords.append((map_point.point.x, map_point.point.y))
        except Exception as e:
            self.get_logger().error(f"TF transform error: {e}")
            return

        # 맵 좌표 출력
        self.get_logger().info(f"Mapped Points: {map_coords}")

        # 시각화 (OpenCV 창)
        visual_map = np.zeros((500, 500, 3), dtype=np.uint8)
        for map_x, map_y in map_coords:
            grid_x = int(map_x * 10 + 250)  # 간단한 맵 스케일링
            grid_y = int(map_y * 10 + 250)
            if 0 <= grid_x < 500 and 0 <= grid_y < 500:
                cv2.circle(visual_map, (grid_x, grid_y), 2, (0, 255, 0), -1)

        cv2.imshow("Mapped Segmentation", visual_map)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = SegmentationToMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
