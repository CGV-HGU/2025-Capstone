#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, MultiArrayDimension
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

        # Publisher & Subscriber
        self.roi_pub = self.create_publisher(Bool, '/floor_detector', 1)
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 1)

        # YOLO 모델 로드
        model_path = "/home/cgv-02/ros2_ws/src/freespace_detection/scripts/best.pt"
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f"YOLO load failed: {e}")
            rclpy.shutdown()
            return

        # Costmap 클리어 서비스 클라이언트 준비
        self.clear_local_cli = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap'
        )
        self.clear_global_cli = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap'
        )
        self.local_service_ready = False
        self.global_service_ready = False

        # Debounce & 타이밍 변수
        self.inference_interval = 0.5  # seconds
        self.last_time = time.time()
        self.in_roi_history = deque(maxlen=5)
        self.confirmed_roi = False

        # ROI 설정 (고정 크기 프레임 기준)
        self.target_w = 320
        self.target_h = 256
        roi_w, roi_h, y_off = 180, 50, 0
        x_c = self.target_w // 2
        y_max = self.target_h - y_off
        y_min = y_max - roi_h
        x_min = x_c - roi_w // 2
        x_max = x_c + roi_w // 2
        self.roi_slice = (slice(y_min, y_max+1), slice(x_min, x_max+1))
        self.roi_area = roi_w * roi_h

        # Costmap clear 조건
        self.true_since = None
        self.cleared_once = False
        self.costmap_obstacle_duration = 5.0  # seconds

        # --- 채널 거리 퍼블리셔 & LUT 초기화 ---
        # 1) 퍼블리셔
        self.channel_pub    = self.create_publisher(
            Float32MultiArray,
            '/lidar_channel_distances',
            10
        )
        # 2) 채널 갯수 및 기본 거리 설정
        self.num_channels   = 141           # 또는 71
        self.range_max      = float('inf')         # 장애물 없을 때 사용
        # (필요하다면) 장애물 있을 때 거리
        self.obstacle_distance = 0.9        

        # 3) 외부에서 생성한 LUT 파일 로드
        #    col_to_ch_lut.npy: shape (W,), dtype=int
        #    distance_lut.npy:  shape (H,), dtype=float
        self.col_to_ch_lut = np.load('/home/cgv-02/ros2_ws/src/freespace_detection/scripts/col_to_ch_lut.npy')
        self.distance_lut  = np.load('/home/cgv-02/ros2_ws/src/freespace_detection/scripts/distance_lut.npy')

        self.bridge = CvBridge()
        self.get_logger().info("Floor Detector Node initialized.")

    def image_callback(self, data):
        now = time.time()
        if now - self.last_time < self.inference_interval:
            return
        self.last_time = now

        try:
            orig_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # 프레임 축소
        small_frame = cv2.resize(
            orig_frame,
            (self.target_w, self.target_h),
            interpolation=cv2.INTER_LINEAR
        )

        # YOLO 추론
        results = self.model(small_frame, stream=True, conf=0.4)
        in_roi = False

        # 첫 번째 마스크 결과 사용
        for res in results:
            if res.masks is None:
                continue

            mask = res.masks.data.cpu().numpy()[0].astype(np.uint8)

            self.publish_channel_distances(mask)
            
            # 세그멘테이션 오버레이 (복사 최소화)
            colored = np.zeros_like(small_frame)
            colored[mask == 1] = (255, 0, 0)
            small_frame = cv2.addWeighted(colored, 0.3, small_frame, 0.7, 0)

            # ROI 내부 채움 비율 계산
            roi_mask = mask[self.roi_slice]
            cnt = cv2.countNonZero(roi_mask)
            if cnt >= self.roi_area * 0.9:
                in_roi = True
            break

        # Debounce
        self.in_roi_history.append(in_roi)
        self.confirmed_roi = sum(self.in_roi_history) >= 2

        # Costmap 클리어 타이밍
        if self.confirmed_roi:
            if self.true_since is None:
                self.true_since = now
            elif not self.cleared_once and (now - self.true_since) >= self.costmap_obstacle_duration:
                self.clear_all_costmaps()
                self.cleared_once = True
        else:
            self.true_since = None
            self.cleared_once = False

        # 결과 퍼블리시data=self.confirmed_roi))
        self.roi_pub.publish(Bool(data=self.confirmed_roi))

        # 디버깅 시각화
        cv2.rectangle(
            small_frame,
            (self.roi_slice[1].start, self.roi_slice[0].start),
            (self.roi_slice[1].stop-1, self.roi_slice[0].stop-1),
            (255, 0, 0), 2
        )
        col = (0, 255, 0) if in_roi else (0, 0, 255)
        cv2.putText(
            small_frame, f"In ROI: {in_roi}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, col, 2
        )

        cv2.imshow("Segmentation Result", small_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def clear_all_costmaps(self):
        # local service
        if not self.local_service_ready:
            if self.clear_local_cli.wait_for_service(timeout_sec=1.0):
                self.local_service_ready = True
            else:
                self.get_logger().warn('local clear service unavailable')
        if self.local_service_ready:
            req = ClearEntireCostmap.Request()
            self.clear_local_cli.call_async(req)

        # global service
        if not self.global_service_ready:
            if self.clear_global_cli.wait_for_service(timeout_sec=1.0):
                self.global_service_ready = True
            else:
                self.get_logger().warn('global clear service unavailable')
        if self.global_service_ready:
            req = ClearEntireCostmap.Request()
            self.clear_global_cli.call_async(req)

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()
        self.get_logger().info("Resources cleaned up.")


    def publish_channel_distances(self, mask: np.ndarray):
        """
        mask: H×W binary mask (1=floor, 0=non-floor)
        Reads precomputed LUTs: self.col_to_ch_lut (shape W,), self.distance_lut (shape H,)
        Publishes Float32MultiArray of size self.num_channels
        """
        H, W = mask.shape
        assert W == self.col_to_ch_lut.shape[0], f"Width mismatch: {W} vs {self.col_to_ch_lut.shape[0]}"
        assert H == self.distance_lut.shape[0], f"Height mismatch: {H} vs {self.distance_lut.shape[0]}"

        # Initialize channel distances to max range
        channel_dist = np.full(self.num_channels, self.range_max, dtype=float)

        # Scan each column for the lowest non-floor pixel
        for x in range(W):
            ys = np.where(mask[:, x] == 0)[0]
            if ys.size > 0:
                y_bot = ys.max()
                dist = float(self.distance_lut[y_bot])
                ch = int(self.col_to_ch_lut[x])
                # Keep the minimum distance for each channel
                if dist < channel_dist[ch]:
                    channel_dist[ch] = dist

        # Build Float32MultiArray message
        msg = Float32MultiArray()
        dim = MultiArrayDimension(label='channels',
                                  size=self.num_channels,
                                  stride=self.num_channels)
        msg.layout.dim.append(dim)
        msg.data = channel_dist.tolist()

        # Publish
        self.channel_pub.publish(msg)

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