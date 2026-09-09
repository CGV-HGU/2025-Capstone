#!/usr/bin/env python3
import os
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

        # scripts 디렉토리 경로 (동적 탐색 및 fallback)
        self.scripts_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
        if not os.path.exists(self.scripts_dir):
            self.scripts_dir = os.path.expanduser('~/ros2_ws/src/freespace_detection/scripts')
        if not os.path.exists(self.scripts_dir):
            self.scripts_dir = os.path.expanduser('~/data/fsd')

        # GUI 활성화 파라미터 및 Headless 환경 자동 감지
        self.declare_parameter('enable_gui', True)
        self.enable_gui = self.get_parameter('enable_gui').get_parameter_value().bool_value
        if not os.environ.get('DISPLAY'):
            self.enable_gui = False
            self.get_logger().info("No DISPLAY detected. Running in headless mode.")

        # 추론 디바이스 파라미터 (Intel Arc iGPU: 'GPU', NPU: 'NPU', CPU: 'CPU')
        self.declare_parameter('inference_device', 'GPU')
        self.inference_device = self.get_parameter('inference_device').get_parameter_value().string_value

        # YOLO 모델 로드 (안정적인 PyTorch .pt 모델 사용)
        pt_path = os.path.join(self.scripts_dir, "best.pt")
        if not os.path.exists(pt_path):
            pt_path = "/home/cgv/data/fsd/best.pt"

        # YOLO Confidence threshold 파라미터 (기본 0.25)
        self.declare_parameter('conf_threshold', 0.25)
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value

        try:
            self.model = YOLO(pt_path, task='segment')
            self.get_logger().info(f"Loaded YOLO model from: {pt_path} (conf={self.conf_threshold})")
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
        self.inference_interval = 0.1  # 10Hz 추론
        self.last_time = time.time()
        self.in_roi_history = deque(maxlen=5)
        self.confirmed_roi = False

        # ROI 설정 (고정 크기 프레임 기준)
        self.target_w = 320
        self.target_h = 256
        roi_w, roi_h, y_off = 180, 50, 5
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
        self.costmap_obstacle_duration = 15.0  # seconds

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
        self.col_to_ch_lut = np.load(os.path.join(self.scripts_dir, 'col_to_ch_lut.npy'))
        dist_2d_path = os.path.join(self.scripts_dir, 'distance_lut_2d.npy')
        if os.path.exists(dist_2d_path):
            self.distance_lut = np.load(dist_2d_path)
            self.is_2d_lut = True
            self.get_logger().info("Using 2D Euclidean distance LUT (range corrected).")
        else:
            self.distance_lut = np.load(os.path.join(self.scripts_dir, 'distance_lut.npy'))
            self.is_2d_lut = False

        # V-LiDAR 스캔 및 벡터 연산용 파라미터
        self.y_min_scan = 120  # 수평선(지평선) 인덱스 (y < 120은 거리 무한대)
        self.row_indices = np.arange(self.y_min_scan, self.target_h)[:, None]

        # Temporal EMA & Persistence 필터 (바닥 반사광 떨림 제거)
        self.alpha = 0.6
        self.clear_persist_frames = 2
        self.filtered_dist = np.full(self.num_channels, float('inf'), dtype=np.float32)
        self.inf_counter = np.zeros(self.num_channels, dtype=int)

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
        try:
            results = self.model(small_frame, stream=False, conf=self.conf_threshold, verbose=False)
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")
            results = []

        # YOLO 마스크 추출 (floor 클래스 = 1)
        mask = None
        for res in results:
            if res.masks is not None and len(res.masks.data) > 0:
                if res.boxes is not None and len(res.boxes) > 0:
                    for i, box in enumerate(res.boxes):
                        cls_id = int(box.cls.item())
                        cls_name = res.names.get(cls_id, '')
                        if cls_id == 1 or cls_name == 'floor':
                            mask = res.masks.data[i].cpu().numpy().astype(np.uint8)
                            break
                if mask is None:
                    mask = res.masks.data[0].cpu().numpy().astype(np.uint8)
                break

        if mask is None:
            # 바닥이 전혀 감지되지 않은 경우 (카메라가 손/장애물/벽 등으로 완전히 가려짐)
            # 100% 장애물(전체 0)로 처리
            mask = np.zeros((self.target_h, self.target_w), dtype=np.uint8)

        # 반사광 및 그림자로 인한 마스크 구멍 보정 (Morphological Close)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # V-LiDAR 141채널 거리 계산 & 퍼블리시
        self.publish_channel_distances(mask)

        # 세그멘테이션 오버레이 (복사 최소화)
        colored = np.zeros_like(small_frame)
        colored[mask == 1] = (255, 0, 0)
        small_frame = cv2.addWeighted(colored, 0.3, small_frame, 0.7, 0)

        # ROI 내부 채움 비율 계산 (/floor_detector 및 Costmap clearing용)
        roi_mask = mask[self.roi_slice]
        cnt = cv2.countNonZero(roi_mask)
        in_roi = (cnt >= self.roi_area * 0.95)

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

        # 결과 퍼블리시
        self.roi_pub.publish(Bool(data=self.confirmed_roi))

        # 디버깅 시각화 (GUI 활성화 시에만 실행)
        if self.enable_gui:
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
            fut = self.clear_local_cli.call_async(req)
            # Release the Future immediately when done
            fut.add_done_callback(lambda f: None)

        # global service
        if not self.global_service_ready:
            if self.clear_global_cli.wait_for_service(timeout_sec=1.0):
                self.global_service_ready = True
            else:
                self.get_logger().warn('global clear service unavailable')
        if self.global_service_ready:
            req = ClearEntireCostmap.Request()
            fut = self.clear_global_cli.call_async(req)
            fut.add_done_callback(lambda f: None)


    def destroy_node(self):
        super().destroy_node()
        if self.enable_gui:
            cv2.destroyAllWindows()
        self.get_logger().info("Resources cleaned up.")

    def publish_channel_distances(self, mask: np.ndarray):
        """
        mask: H×W binary mask (1=floor, 0=non-floor/obstacle)
        전체 가로 폭(x in [0, W))에 대해 수평선부터 로봇 최하단까지 스캔하여 장애물 거리를 계산합니다.
        """
        channel_dist = np.full(self.num_channels, self.range_max, dtype=np.float32)

        # 수평선(y_min_scan)부터 바닥 최하단(target_h)까지 스캔
        sub_mask = mask[self.y_min_scan:, :]
        obs_rows = np.where(sub_mask == 0, self.row_indices, -1)
        max_y = obs_rows.max(axis=0)  # shape (W,), 각 열별 최하단(로봇에 가장 가까운) 장애물 픽셀 행

        valid_cols = np.where(max_y >= 0)[0]
        if valid_cols.size > 0:
            y_pts = max_y[valid_cols]
            if self.is_2d_lut:
                dists = self.distance_lut[y_pts, valid_cols]
            else:
                dists = self.distance_lut[y_pts]
            chs = self.col_to_ch_lut[valid_cols]
            # 각 채널별 최소 거리 집계
            np.minimum.at(channel_dist, chs, dists)

        # Temporal EMA & Persistence 필터 적용 (바닥 반사광 떨림 방지)
        filtered_dist = self.apply_temporal_filter(channel_dist)

        # 메시지 빌드 및 퍼블리시
        msg = Float32MultiArray()
        dim = MultiArrayDimension(label='channels',
                                size=self.num_channels,
                                stride=self.num_channels)
        msg.layout.dim.append(dim)
        msg.data = filtered_dist.tolist()
        self.channel_pub.publish(msg)

    def apply_temporal_filter(self, raw_dist: np.ndarray) -> np.ndarray:
        """
        시간적 EMA 필터 및 지속성 검증으로 반사광 노이즈 제거:
        1. 신규 장애물 진입 (raw=유한, prev=inf): 충돌 방지 위해 즉각 반응
        2. 장애물 추적 (raw=유한, prev=유한): EMA 스무딩 (alpha=0.6)
        3. 장애물 소멸 (raw=inf, prev=유한): 2연속 프레임 확인 후 inf 전환 (플리커 제거)
        4. 지속적 Freespace (raw=inf, prev=inf): inf 유지
        """
        raw_finite = np.isfinite(raw_dist)
        prev_finite = np.isfinite(self.filtered_dist)

        # 1) 추적 중인 장애물 -> EMA 스무딩
        both_finite = raw_finite & prev_finite
        self.filtered_dist[both_finite] = (
            self.alpha * raw_dist[both_finite] + (1.0 - self.alpha) * self.filtered_dist[both_finite]
        )
        self.inf_counter[raw_finite] = 0

        # 2) 신규 장애물 등장 -> 즉각 반영
        new_obs = raw_finite & (~prev_finite)
        self.filtered_dist[new_obs] = raw_dist[new_obs]
        self.inf_counter[new_obs] = 0

        # 3) 장애물 소멸 감지 -> Persistence 체크
        disappeared = (~raw_finite) & prev_finite
        self.inf_counter[disappeared] += 1
        to_clear = disappeared & (self.inf_counter >= self.clear_persist_frames)
        self.filtered_dist[to_clear] = float('inf')

        # 4) 빈 공간 유지
        both_inf = (~raw_finite) & (~prev_finite)
        self.filtered_dist[both_inf] = float('inf')

        return self.filtered_dist.copy()

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