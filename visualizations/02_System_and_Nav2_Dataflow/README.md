# ⚡ 02. 시스템 및 Nav2 연동 파이프라인 시각화 (System & Nav2 Architecture)

본 디렉터리는 단안 카메라 영상이 인공지능 바닥 분할을 거쳐 가상 라이다(`/scan`) 및 Nav2 코스트맵, 모터 제어(`/cmd_vel`)로 전달되는 **전체 ROS 2 데이터 파이프라인과 파라미터 구조**를 시각화한 자료입니다.

---

## 📊 시각화 자료 목록

### 1. `01_V-LiDAR_to_Nav2_Complete_Pipeline.png`
* **전체 노드 및 토픽 연동 흐름**:
  1. `image_tools (cam2image)`: `/dev/video0` ➔ `camera/image_raw` ($640 \times 480$ @ 30 FPS)
  2. `freespace_detection`: YOLO11n-seg OpenVINO ➔ `/lidar_channel_distances` (141개 채널)
  3. `fake_lidar_with_tf`: 2D 기하 LUT 변환 ➔ `/scan` (`sensor_msgs/LaserScan`, FOV $\pm 35^\circ$)
  4. `stella_vslam_ros` & `pose_converter`: 위치 추정 ➔ `map -> odom` TF 변환 발행
  5. `omo_r1_navigation2`: Global Costmap (`NTH4F.png`) + Local Costmap (`/scan`) ➔ DWB Controller
  6. `omo_r1_bringup`: `/cmd_vel` 수신 ➔ `/dev/ttyMCU` 양쪽 차동 모터 구동

### 2. `02_Nav2_Costmap_Marking_and_Raytracing.png`
* **로컬 코스트맵 파라미터 시각화 ([`omo_r1.yaml`](file:///home/cgv/ros2_ws/src/omo_r1_navigation2/param/omo_r1.yaml))**:
  * **로봇 차체 Footprint**: $0.57\text{m} \times 0.57\text{m}$ (차폭 기준)
  * **인플레이션 반경 (Inflation Radius)**: $0.55\text{m}$ (안전 마진)
  * **장애물 마킹 영역 (Obstacle Marking)**: $0.0\text{m} \sim 3.0\text{m}$
  * **실시간 레이트레이싱 클리어 영역 (Raytracing Clearing)**: $0.0\text{m} \sim 3.5\text{m}$ (FOV $\pm 35^\circ$)
  * **원리**: 장애물이 치워졌을 때 $3.5\text{m}$ 콘(Cone) 영역 내의 이전 장애물 잔상이 실시간으로 삭제되는 기법 설명.
