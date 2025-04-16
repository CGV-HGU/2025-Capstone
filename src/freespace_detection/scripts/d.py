#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import numpy as np


class RoiChecker(Node):
    def __init__(self):
        super().__init__('floor_detector')

        # Publisher 및 Subscriber 설정
        self.roi_pub = self.create_publisher(Bool, '/floor_detector', 10)
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)

        # YOLO 모델 로드
        model_path = "/home/cgv-02/ros2_ws/src/freespace_detection/scripts/best.pt"
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f"YOLO load failed: {e}")
            rclpy.shutdown()
            return

        # CV Bridge 초기화
        self.bridge = CvBridge()
        self.get_logger().info("Floor Detector Node initialized.")

    def image_callback(self, data):
        # 1) ROS→OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # 2) 프레임 크기
        height, width, _ = frame.shape

        # 3) ROI 크기 및 위치 (하단 중앙)
        roi_w, roi_h = 40, 10               # ROI 너비 40px, 높이 10px
        x_center = width // 2
        y_bottom_offset = 50                # 아래에서 50px 위
        x_min = x_center - roi_w // 2
        x_max = x_center + roi_w // 2
        y_max = height - y_bottom_offset
        y_min = y_max - roi_h

        # 4) ROI 테두리 그리기 (파란색, 두께=2)
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)

        # 5) YOLO 추론
        results = self.model(frame, stream=True, conf=0.4)
        in_roi_80 = False

        for result in results:
            if result.masks is None:
                continue

            masks = result.masks.data.cpu().numpy()
            for mask in masks:
                # ROI 내부 마스크 픽셀만 셈
                roi_mask = mask[y_min:y_max+1, x_min:x_max+1]
                count = int(np.count_nonzero(roi_mask))
                area = roi_w * roi_h

                if count >= area * 0.8:
                    in_roi_80 = True

                    # ROI 전체를 반투명 파란색으로 칠함
                    overlay = frame.copy()
                    overlay[y_min:y_max+1, x_min:x_max+1] = (255, 0, 0)
                    cv2.addWeighted(overlay, 0.3, frame, 0.7, 0, frame)
                    break
            if in_roi_80:
                break

        # 6) 결과 퍼블리시
        self.roi_pub.publish(Bool(data=in_roi_80))

        # 7) 디버깅 텍스트
        color = (0, 255, 0) if in_roi_80 else (0, 0, 255)
        cv2.putText(frame, f"In ROI 80% or more: {in_roi_80}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        # 8) 화면 출력
        cv2.imshow("Segmentation Result", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()
        self.get_logger().info("Resources cleaned up.")

def main(args=None):
    rclpy.init(args=args)
    node = RoiChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
