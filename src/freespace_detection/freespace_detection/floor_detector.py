#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import numpy as np

class floorDetector(Node):
    def __init__(self):
        super().__init__('floor_detector')

        # Publisher (감지 결과 퍼블리시)
        self.roi_pub = self.create_publisher(Bool, '/floor_detector', 10)

        # Subscriber
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)

        # YOLO 모델 로드
        model_path = "/home/cgv/ros2_ws/src/freespace_detection/scripts/best.pt"
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            rclpy.shutdown()
            return

        self.bridge = CvBridge()
        self.get_logger().info("Floor Detector Node initialized.")

    def image_callback(self, data):
        try:
            # ROS Image 메시지를 OpenCV 이미지로 변환
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # YOLO 모델 실행
        results = self.model(frame, stream=True, conf=0.6)
        
        # 원본 프레임을 복사하여 overlay 이미지 생성
        overlay = frame.copy()

        # 결과 처리
        for result in results:
            if result.masks is not None:
                # 마스크 배열의 해상도는 원본 이미지와 다를 수 있으므로, 먼저 리사이즈합니다.
                masks = result.masks.data.cpu().numpy()  # shape: (n, mask_h, mask_w)
                for mask in masks:
                    # float 혹은 bool 배열일 수 있으므로, uint8로 변환 후 리사이즈
                    mask_uint8 = (mask > 0.6).astype(np.uint8)
                    # 원본 이미지 크기로 리사이즈 (최근접 보간법 사용)
                    mask_resized = cv2.resize(mask_uint8, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
                    # boolean mask로 변환
                    bool_mask = mask_resized.astype(bool)

                    # overlay 이미지에서 세그멘테이션 영역을 파란색 (BGR: [255, 0, 0])으로 채움
                    overlay[bool_mask] = [255, 0, 0]

        # alpha 값을 조정하여 투명도 설정 (0.5는 50% 투명)
        alpha = 0.5
        blended = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

        # 퍼블리시 (여기서는 단순히 True를 퍼블리시)
        self.roi_pub.publish(Bool(data=True))

        # 결과 이미지 창에 표시
        cv2.imshow("Detection Result", blended)
        cv2.waitKey(1)  # 창 업데이트를 위한 대기

def main(args=None):
    rclpy.init(args=args)
    floor_detector = floorDetector()
    try:
        rclpy.spin(floor_detector)
    except KeyboardInterrupt:
        pass
    finally:
        floor_detector.destroy_node()
        cv2.destroyAllWindows()  # 모든 OpenCV 창 종료
        rclpy.shutdown()

if __name__ == '__main__':
    main()
