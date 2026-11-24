#!/usr/bin/env python3
import math
import os
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Float32MultiArray, Header, MultiArrayDimension
from ultralytics import YOLO


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

        # 추론 백엔드 및 디바이스 파라미터 (OpenVINO FP16 CPU 기본, 필요 시 파라미터 전환)
        self.declare_parameter('use_openvino', True)
        self.use_openvino = self.get_parameter('use_openvino').get_parameter_value().bool_value
        self.declare_parameter('inference_device', 'CPU')
        self.inference_device = self.get_parameter('inference_device').get_parameter_value().string_value

        # YOLO Confidence & 클래스 파라미터
        # NOTE: best.pt / best_openvino_model 메타데이터(names={0:'person', 1:'floor'})와 달리
        # 실제 학습된 바닥 폴리곤 라벨은 cls=0으로 예측됨(cls=1 미검출).
        self.declare_parameter('conf_threshold', 0.15)
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.declare_parameter('floor_class_id', 0)
        self.floor_class_id = self.get_parameter('floor_class_id').get_parameter_value().integer_value
        self.declare_parameter('min_floor_scan_pixels', 50)
        self.min_floor_scan_pixels = self.get_parameter('min_floor_scan_pixels').get_parameter_value().integer_value

        # Costmap 자동 전체 삭제 기본 비활성화 (사각지대 장애물 기억 삭제 방지)
        self.declare_parameter('enable_auto_costmap_clear', False)
        self.enable_auto_costmap_clear = self.get_parameter('enable_auto_costmap_clear').get_parameter_value().bool_value

        # LaserScan 직접 발행 파라미터 (원본 이미지 header.stamp 보존)
        self.declare_parameter('publish_scan_directly', True)
        self.publish_scan_directly = self.get_parameter('publish_scan_directly').get_parameter_value().bool_value
        self.declare_parameter('scan_topic', '/scan')
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.declare_parameter('lidar_frame_id', 'lidar_link')
        self.lidar_frame_id = self.get_parameter('lidar_frame_id').get_parameter_value().string_value

        self.model, self.backend_label, self.predict_device = self._load_yolo_model()
        if self.model is None:
            rclpy.shutdown()
            return

        # Costmap 클리어 서비스 클라이언트 준비 (opt-in)
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
        self.roi_slice = (slice(y_min, y_max + 1), slice(x_min, x_max + 1))
        self.roi_area = roi_w * roi_h

        # Costmap clear 조건
        self.true_since = None
        self.cleared_once = False
        self.costmap_obstacle_duration = 15.0  # seconds

        # --- 채널 거리 / LaserScan 퍼블리셔 & LUT 초기화 ---
        self.channel_pub = self.create_publisher(
            Float32MultiArray,
            '/lidar_channel_distances',
            10
        )
        self.scan_pub = self.create_publisher(
            LaserScan,
            self.scan_topic,
            10
        ) if self.publish_scan_directly else None

        self.num_channels = 141
        self.angle_min = math.radians(-35.0)
        self.angle_max = math.radians(35.0)
        self.angle_increment = math.radians(0.5)
        self.scan_range_min = 0.1
        self.scan_range_max = 10.0
        self.range_max = float('inf')  # 장애물 없을 때 사용
        self.obstacle_distance = 0.9

        # 외부에서 생성한 LUT 파일 로드
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

        # Temporal EMA & Persistence 필터 (바닥 반사광 떨림 및 유령 장애물 완전 제거)
        self.alpha = 0.6
        self.clear_persist_frames = 2  # 장애물 소멸 지속 프레임 (깜빡임 방지)
        self.obs_persist_frames = 2    # 신규 장애물 확인 프레임 (1프레임 순간 반사 노이즈 무시)
        self.filtered_dist = np.full(self.num_channels, float('inf'), dtype=np.float32)
        self.inf_counter = np.zeros(self.num_channels, dtype=int)
        self.obs_counter = np.zeros(self.num_channels, dtype=int)

        self.bridge = CvBridge()
        self.get_logger().info(
            f"Floor Detector Node initialized ({self.backend_label}, "
            f"floor_cls={self.floor_class_id}, direct_scan={self.publish_scan_directly}, "
            f"auto_clear={self.enable_auto_costmap_clear})."
        )

    def _resolve_openvino_device(self, requested: str) -> str:
        req = (requested or 'CPU').strip().upper()
        try:
            import openvino as ov
            avail = [d.upper() for d in ov.Core().available_devices]
        except Exception:
            avail = ['CPU']
        if req in avail:
            return 'cpu' if req == 'CPU' else f'intel:{req.lower()}'
        self.get_logger().warn(
            f"Requested OpenVINO device '{requested}' unavailable (available={avail}); falling back to CPU."
        )
        return 'cpu'

    def _load_yolo_model(self):
        ov_dir = os.path.join(self.scripts_dir, "best_openvino_model")
        if not os.path.exists(ov_dir):
            ov_dir = "/home/cgv/data/fsd/best_openvino_model"

        pt_path = os.path.join(self.scripts_dir, "best.pt")
        if not os.path.exists(pt_path):
            pt_path = "/home/cgv/data/fsd/best.pt"

        if self.use_openvino and os.path.exists(ov_dir):
            try:
                dev = self._resolve_openvino_device(self.inference_device)
                model = YOLO(ov_dir, task='segment')
                self.get_logger().info(
                    f"Loaded OpenVINO FP16 model from: {ov_dir} (device={dev}, conf={self.conf_threshold})"
                )
                return model, f"OpenVINO({dev})", dev
            except Exception as e:
                self.get_logger().warn(f"OpenVINO load failed ({e}); falling back to PyTorch .pt")

        try:
            model = YOLO(pt_path, task='segment')
            self.get_logger().info(
                f"Loaded PyTorch model from: {pt_path} (device=cpu, conf={self.conf_threshold})"
            )
            return model, "PyTorch(cpu)", "cpu"
        except Exception as e:
            self.get_logger().error(f"YOLO load failed: {e}")
            return None, "none", "cpu"

    def _extract_floor_mask(self, results) -> np.ndarray:
        """Select only `floor_class_id` masks connected to the ground scan band (y >= y_min_scan)."""
        floor_masks = []
        for res in results:
            if res.masks is None or len(res.masks.data) == 0:
                continue
            if res.boxes is not None and len(res.boxes) == len(res.masks.data):
                cls_ids = res.boxes.cls.cpu().numpy().astype(int)
                for m, cls_id in zip(res.masks.data, cls_ids):
                    if cls_id != self.floor_class_id:
                        continue
                    m_bin = (m.cpu().numpy() > 0.5).astype(np.uint8)
                    # 상단 배경(y < y_min_scan)에만 뜬 고립 오탐 차단
                    if int(m_bin[self.y_min_scan:, :].sum()) < self.min_floor_scan_pixels:
                        continue
                    floor_masks.append(m_bin)
            break

        if floor_masks:
            return np.bitwise_or.reduce(floor_masks)
        return np.zeros((self.target_h, self.target_w), dtype=np.uint8)

    def image_callback(self, data):
        now = time.time()
        if now - self.last_time < self.inference_interval:
            return
        self.last_time = now

        # 원본 카메라 획득 타임스탬프 보존 (비어 있으면 현재 ROS 시각 사용)
        acq_stamp = data.header.stamp
        if acq_stamp.sec == 0 and acq_stamp.nanosec == 0:
            acq_stamp = self.get_clock().now().to_msg()

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
            results = self.model(
                small_frame,
                stream=False,
                conf=self.conf_threshold,
                device=self.predict_device,
                verbose=False
            )
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")
            results = []

        mask = self._extract_floor_mask(results)

        # 반사광(천장 전등 및 사물 반사) 및 그림자로 인한 마스크 구멍 보정 (7x7, 2회 반복)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        # V-LiDAR 141채널 거리 계산 & 퍼블리시 (원본 획득 타임스탬프 유지)
        self.publish_channel_distances(mask, acq_stamp)

        # 세그멘테이션 오버레이 (복사 최소화)
        colored = np.zeros_like(small_frame)
        colored[mask == 1] = (255, 0, 0)
        small_frame = cv2.addWeighted(colored, 0.3, small_frame, 0.7, 0)

        # ROI 내부 채움 비율 계산 (/floor_detector용)
        roi_mask = mask[self.roi_slice]
        cnt = cv2.countNonZero(roi_mask)
        in_roi = (cnt >= self.roi_area * 0.95)

        # Debounce
        self.in_roi_history.append(in_roi)
        self.confirmed_roi = sum(self.in_roi_history) >= 2

        # Costmap 자동 전체 삭제는 기본 비활성화(opt-in)
        if self.enable_auto_costmap_clear:
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
                (self.roi_slice[1].stop - 1, self.roi_slice[0].stop - 1),
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

    def publish_channel_distances(self, mask: np.ndarray, acq_stamp=None):
        """
        mask: H×W binary mask (1=floor, 0=non-floor/obstacle)
        전체 가로 폭(x in [0, W))에 대해 수평선부터 로봇 최하단까지 스캔하여 장애물 거리를 계산합니다.
        """
        if acq_stamp is None:
            acq_stamp = self.get_clock().now().to_msg()

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
            np.minimum.at(channel_dist, chs, dists)

        # Temporal EMA & Persistence 필터 적용 (바닥 반사광 떨림 방지)
        filtered_dist = self.apply_temporal_filter(channel_dist)

        # 채널 거리 토픽 퍼블리시 (label에 획득 시각 ns를 인코딩하여 downstream 만료 검사 지원)
        stamp_ns = int(acq_stamp.sec) * 1_000_000_000 + int(acq_stamp.nanosec)
        msg = Float32MultiArray()
        dim = MultiArrayDimension(
            label=f'channels@{stamp_ns}',
            size=self.num_channels,
            stride=self.num_channels
        )
        msg.layout.dim.append(dim)
        msg.data = filtered_dist.tolist()
        self.channel_pub.publish(msg)

        # 원본 획득 타임스탬프가 유지된 LaserScan 직접 발행 (stale scan 재발행 원천 차단)
        if self.scan_pub is not None:
            scan = LaserScan()
            scan.header = Header(stamp=acq_stamp, frame_id=self.lidar_frame_id)
            scan.angle_min = self.angle_min
            scan.angle_max = self.angle_max
            scan.angle_increment = self.angle_increment
            scan.range_min = self.scan_range_min
            scan.range_max = self.scan_range_max
            scan.scan_time = self.inference_interval
            scan.time_increment = 0.0
            scan.ranges = filtered_dist[::-1].tolist()
            scan.intensities = [0.0] * self.num_channels
            self.scan_pub.publish(scan)

    def apply_temporal_filter(self, raw_dist: np.ndarray) -> np.ndarray:
        """
        시간적 EMA 필터 및 양방향 지속성 검증으로 반사광 노이즈 완전 제거:
        1. 신규 장애물 진입: 2프레임 연속 검출 시에만 등록 (1프레임 순간 반사광 노이즈 무시)
        2. 장애물 추적: EMA 스무딩 (alpha=0.6)
        3. 장애물 소멸: 2프레임 연속 미검출 시에만 inf 전환 (플리커링 방지)
        4. 지속적 Freespace: inf 유지
        """
        raw_finite = np.isfinite(raw_dist)
        prev_finite = np.isfinite(self.filtered_dist)

        # 장애물 감지 연속 카운터 갱신
        self.obs_counter[raw_finite] += 1
        self.obs_counter[~raw_finite] = 0

        # 장애물 미감지(inf) 연속 카운터 갱신
        self.inf_counter[~raw_finite] += 1
        self.inf_counter[raw_finite] = 0

        # 1) 이미 추적 중인 장애물 유지 및 EMA 스무딩
        both_finite = raw_finite & prev_finite
        self.filtered_dist[both_finite] = (
            self.alpha * raw_dist[both_finite] + (1.0 - self.alpha) * self.filtered_dist[both_finite]
        )

        # 2) 신규 장애물 진입: 2프레임 연속 검출 시에만 등록 (1프레임 순간 반사광 노이즈 필터링)
        new_confirmed = (~prev_finite) & (self.obs_counter >= self.obs_persist_frames)
        self.filtered_dist[new_confirmed] = raw_dist[new_confirmed]

        # 3) 장애물 소멸 감지: 2프레임 연속 미검출 시에만 inf로 전환 (플리커링 방지)
        to_clear = prev_finite & (self.inf_counter >= self.clear_persist_frames)
        self.filtered_dist[to_clear] = float('inf')

        # 4) 빈 공간 유지 (확인되지 않은 1프레임 신규 노이즈 포함)
        still_inf = (~prev_finite) & (~new_confirmed)
        self.filtered_dist[still_inf] = float('inf')

        return self.filtered_dist.copy()


def main(args=None):
    rclpy.init(args=args)
    node = RoiChecker()
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
