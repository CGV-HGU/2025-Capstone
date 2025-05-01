#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav2_msgs.srv import ClearEntireCostmap
from collections import deque
import cv2
from ultralytics import YOLO
import numpy as np

class RoiChecker(Node):
    def __init__(self):
        super().__init__('floor_detector')

        # 1) Publisher & Subscriber
        self.roi_pub = self.create_publisher(Bool, '/floor_detector', 1)
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 1)

        # 2) YOLO 모델 로드 (실제 경로로 수정)
        model_path = "/home/cgv-02/ros2_ws/src/freespace_detection/scripts/best.pt"
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f"YOLO load failed: {e}")
            rclpy.shutdown()
            return

        # 1) Costmap 클리어 서비스 클라이언트 준비
        #    노드 네임스페이스나 설정에 따라 서비스 이름이 다를 수 있음
        self.clear_local_cli = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap'
        )
        self.clear_global_cli = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap'
        )

        # 2) true 지속 시간 추적 변수
        self.true_since = None
        self.cleared_once = False
        self.costmap_obstacle_duration = 5.0 #3 sec

        # 3) CV Bridge 초기화
        self.bridge = CvBridge()
        self.get_logger().info("Floor Detector Node initialized.")

        self.inference_interval = 0.0  # 원하는 inference 주기 (초)
        self.last_inference_time = self.get_clock().now()

        self.in_roi_history = deque(maxlen=5)
        self.confirmed_roi = False  # 5회 중 3회 이상 in_roi이면 True

    def image_callback(self, data):
        now = self.get_clock().now()
        elapsed = (now - self.last_inference_time).nanoseconds * 1e-9
        if elapsed < self.inference_interval:
            return
        self.last_inference_time = now

        #  ROS Image → OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # YOLO 추론
        results = self.model(frame, stream=True, conf=0.4)
        in_roi = False

        # 프레임 크기
        h, w, _ = frame.shape

        # ROI 크기 & 위치 (하단 중앙)
        roi_w, roi_h = 180, 20
        x_c = w // 2
        y_off = 20
        x_min = x_c - roi_w // 2
        x_max = x_c + roi_w // 2
        y_max = h - y_off
        y_min = y_max - roi_h

        for res in results:
            if res.masks is None:
                continue

            # (1) 모델이 리사이즈한 작은 마스크 얻기
            mask_small = res.masks.data.cpu().numpy()[0].astype(np.uint8)
            # (2) 원본 프레임 크기로 다시 확대
            mask = cv2.resize(mask_small, (w, h), interpolation=cv2.INTER_NEAREST)

            # (3) 전체 세그멘테이션 오버레이
            overlay = frame.copy()
            overlay[mask == 1] = (255, 0, 0)
            frame = cv2.addWeighted(overlay, 0.3, frame, 0.7, 0)

            # (4) ROI 내부 채움 비율 계산
            roi_mask = mask[y_min:y_max+1, x_min:x_max+1]
            cnt = int(np.count_nonzero(roi_mask))
            if cnt >= roi_w * roi_h * 0.8:
                in_roi = True
            break  # 첫 번째 마스크만 사용

        # --- 추가: history 업데이트 및 confirmed_roi 계산
        self.in_roi_history.append(in_roi)
        # 히스토리가 아직 채워지지 않아도, 들어간 값 중 2 이상 True면 confirmed
        self.confirmed_roi = sum(self.in_roi_history) >= 2

        # confirmed_roi 상태에 따른 시간 추적
        if self.confirmed_roi:
            if self.true_since is None:
                self.true_since = now
            else:
                elapsed = (now - self.true_since).nanoseconds * 1e-9
                if not self.cleared_once and elapsed >= self.costmap_obstacle_duration:
                    self.clear_all_costmaps()
                    self.cleared_once = True  # ← 플래그 설정
        else:
            # false가 들어오면 타이머 리셋
            self.true_since = None
            self.cleared_once = False


        # 결과 퍼블리시
        self.roi_pub.publish(Bool(data=self.confirmed_roi))

        # ROI 테두리 그리기
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)

        # 디버깅 텍스트
        col = (0,255,0) if in_roi else (0,0,255)
        cv2.putText(frame, f"In ROI : {in_roi}",
                    (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, col, 2)

        # 화면 출력
        cv2.imshow("Segmentation Result", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()
        self.get_logger().info("Resources cleaned up.")


    def clear_all_costmaps(self):
        for cli, name in [(self.clear_local_cli, 'local'), (self.clear_global_cli, 'global')]:
            if not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'{name} clear service unavailable')
                continue
            req = ClearEntireCostmap.Request()
            future = cli.call_async(req)
            future.add_done_callback(lambda f, n=name: self._on_clear_done(f, n))

    def _on_clear_done(self, future, which):
        if future.result() is None:
            self.get_logger().error(f'{which} costmap clear failed')
        else:
            self.get_logger().info(f'{which} costmap cleared')


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
