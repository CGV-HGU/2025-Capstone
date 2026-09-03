# 🤖 Vision-Based 2D Scan Generation (V-LiDAR) for AMR Obstacle Avoidance

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Robot-OMORobot_R1_Mini-orange.svg)](https://github.com/omorobot/omo_r1-ros2)
[![Vision](https://img.shields.io/badge/AI_Model-YOLOv11n--seg-green.svg)](https://docs.ultralytics.com/models/yolo11/)
[![License](https://img.shields.io/badge/License-Apache_2.0-lightgrey.svg)](LICENSE)

본 프로젝트는 고가의 2D/3D LiDAR 센서 없이, **단안(Monocular) RGB 카메라 1대와 AI 바닥 분할(Floor Segmentation)** 기법을 활용하여 실시간 **가상 2D LaserScan(V-LiDAR) 데이터를 생성**하고, 이를 **ROS 2 Nav2 내비게이션 스택과 연동하여 실내 자율 장애물 회피 주행**을 수행하는 통합 로보틱스 시스템입니다.

---

## 📌 1. 시스템 개요 (System Overview)

- **연구 논문**: *Vision-Based 2D Scan Generation for Obstacle Avoidance Using Floor Segmentation* (Handong Global Univ., 2025)
- **로봇 플랫폼**: OMO-R1 차동 구동 모바일 로봇 (Differential Drive Mobile Robot)
- **핵심 기술**:
  1. **경량 AI 바닥 분할**: `YOLOv11n-seg` 기반 실시간 주행 가능 바닥(Freespace) 및 비바닥(Obstacle) 영역 분할 ($320 \times 256$ 해상도, ~100ms)
  2. **기하학적 Look-Up Table (LUT)**: 카메라 캘리브레이션 파라미터(높이 $1.05\text{m}$, 틸트 $2.0^\circ$, 화각 $\pm 35^\circ$) 기반 초고속 $O(1)$ 각도/거리 변환
  3. **가상 2D 스캔 변환 (V-LiDAR)**: 각 열의 최하단 Non-floor 접촉점 추출 ➔ 141개 채널($0.5^\circ$ 간격) `sensor_msgs/LaserScan` 토픽(`/scan`) 발행
  4. **Visual SLAM & Nav2 연동**: `stella-vslam` 기반 2D 위치 추정 및 Nav2 Costmap 실시간 장애물 회피 경로 생성

```
[ 단안 USB 카메라 ] 
       │ (camera/image_raw)
       ├──► [ freespace_detection ] (YOLOv11n-seg + LUTs)
       │           │ (/lidar_channel_distances, 141ch)
       │           ▼
       │    [ fake_lidar_with_tf ] ──► (/scan: sensor_msgs/LaserScan)
       │                                     │
       │                                     ▼
       └──► [ stella_vslam_ros ]      [ omo_r1_navigation2 (Nav2) ]
                   │                         │ (/cmd_vel)
                   ▼                         ▼
            [ pose_converter ]        [ omo_r1_bringup (MCU) ]
                   │                         │
            (TF: map -> odom)           [ 모터 구동 ]
```

---

## 📊 2. 팩트 기반 시각화 대시보드 (Fact-Based Visualizations)

로봇 워크스페이스 내의 실제 소스코드, 맵 파일, VSLAM 키프레임 궤적, Nav2 파라미터의 **실측 수치(Ground Truth)만을 기반으로 제작된 정밀 시각화 자료 모음**입니다. 상세 설명은 [`visualizations/README.md`](visualizations/README.md)에서 확인하실 수 있습니다.

| 대분류 | 시각화 항목 | 상세 내용 | 바로가기 |
| :--- | :--- | :--- | :---: |
| **01. 주행 맵 & 좌표계** | • `01_NTH4F_Nav2_2D_Occupancy_Grid_Map.png`<br>• `02_OH3F_Visual_SLAM_Trajectory_Reconstruction.png`<br>• `03_Map_Resolution_and_Scale_Evolution.png` | • 뉴턴홀 4층 2D 격자 및 미터 좌표계 투영 (90m x 27.55m)<br>• 오석관 3층 VSLAM 키프레임 2D 궤적 복원 (132개 Keyframe)<br>• 6개 2D 맵 변천사 및 해상도 비교 ($0.05\text{m/cell}$) | [폴더 열기](visualizations/01_Maps_and_Coordinates/README.md) |
| **02. 시스템 & Nav2 연동** | • `01_V-LiDAR_to_Nav2_Complete_Pipeline.png`<br>• `02_Nav2_Costmap_Marking_and_Raytracing.png` | • 카메라 ➔ AI ➔ V-LiDAR ➔ Nav2 전체 ROS 2 데이터 파이프라인<br>• 로컬 코스트맵 장애물 마킹(3m) & 클리어링(3.5m) 반경도 | [폴더 열기](visualizations/02_System_and_Nav2_Dataflow/README.md) |
| **03. AI 및 센서 벤치마크** | • `01_YOLO11_Inference_Latency_and_FPS_Benchmark.png`<br>• `02_V-LiDAR_141Ch_Sector_Distribution.png` | • OpenVINO (78.4 FPS / 12.8ms) vs PyTorch 실측 벤치마크<br>• 141개 채널 3구역(좌/중/우) 각도 분할도 | [폴더 열기](visualizations/03_Sensor_and_AI_Benchmark/README.md) |

---

## 📂 3. 패키지 구성 및 설명 (Workspace Packages)

`ros2_ws/src/` 디렉터리 내 주요 15개 ROS 2 패키지의 기능과 역할입니다:

| 구분 | 패키지명 | 언어 | 상세 설명 |
| :--- | :--- | :---: | :--- |
| **V-LiDAR<br>& 비전** | **`freespace_detection`** | Python | • `YOLOv11n-seg` 기반 바닥 분할 추론<br>• `col_to_ch_lut.npy`, `distance_lut.npy` 기반 거리 계산<br>• 전방 ROI 바닥 지속 시 Costmap Clear 서비스 호출 |
| | **`fake_lidar_with_tf`** | Python | • 141개 채널 거리 데이터를 `sensor_msgs/LaserScan`(`/scan`)으로 변환<br>• `base_link -> lidar_link` TF 브로드캐스트 |
| | **`image_tools`** | C++ | • USB 웹캠 장치(`/dev/video0`)로부터 실시간 영상 캡처 및 `camera/image_raw` 토픽 발행 (`cam2image`) |
| | **`segmentation_to_map`** | Python | • 세그멘테이션 마스크를 카메라 인트린식 및 TF를 통해 맵 좌표계로 3D 프로젝션하는 노드 |
| **SLAM<br>& 위치추정** | **`stella_vslam_ros`** | C++ | • 단안 카메라 기반 Visual SLAM (`run_slam`)<br>• 사전 구축된 지도(`NTH4F.msg`) 및 FBoW 사전(`orb_vocab.fbow`) 기반 위치 추정 |
| | **`pose_converter`** | C++ | • VSLAM 카메라 포즈(`/run_slam/camera_pose`)를 수신하여 Nav2용 `map -> odom` TF 변환 발행<br>• Tracking 성공 시 `reset_odom` 호출 |
| **로봇 제어<br>& 하드웨어** | **`omo_r1_bringup`** | Python | • OMO-R1 로봇 MCU 통신 노드 (`omo_r1_mcu_node`)<br>• 엔코더 기반 오도메트리(`/odom`), TF(`odom -> base_link`), 배터리/LED 서비스 제공 |
| | **`omo_r1_description`** | URDF | • OMO-R1 3D 로봇 모델(URDF), 링크 및 바퀴/라이다 STL 메쉬 정의 |
| | **`omo_r1_interfaces`** | Msg/Srv | • 로봇 제어용 커스텀 서비스 정의 (`ResetOdom.srv`, `Battery.srv`, `Onoff.srv`, `Color.srv` 등) |
| | **`omo_r1_teleop`** | Python | • 키보드를 통한 수동 원격 조종 패키지 (`teleop_keyboard`) |
| | **`omo_r1_cartographer`** | Config | • 2D 라이다 매핑을 위한 Google Cartographer SLAM 설정 |
| | **`omo_r1`** | Meta | • OMO-R1 관련 패키지들을 묶어주는 메타 패키지 |
| **내비게이션<br>& 통합 실행**| **`omo_r1_navigation2`**| YAML | • Nav2 DWB Controller, Costmap, Recovery 파라미터(`omo_r1.yaml`) 및 맵(`NTH4F.png`, `map.yaml`) 관리 |
| | **`robot_launch_package`**| Python | • 카메라 + MCU + Freespace + Fake Lidar + Nav2 + RViz를 한 번에 기동하는 통합 런치 (`multi_node_launch.py`) |
| | **`network_control`** | Python | • Supabase 클라우드 데이터베이스 연동 원격 목적지 수신 및 로봇 상태 모니터링 노드 |

---

## ⚙️ 3. 시스템 사양 및 데이터 파일 구성 (System Specifications & Data Files)

### 1) 로봇 및 센서 구성 (Robot & Sensor Hardware)
- **모바일 로봇 플랫폼**: OMORobot R1 Mini (차동 구동 모바일 로봇, 시리얼 포트 `/dev/ttyUSB0` ➔ `/dev/ttyMCU`, `/dev/ttyMotor`)
- **비전 센서**: Realtek HD USB 웹캠 (`/dev/video0`, 전방 장착, 지면 높이 1.05m, 아래쪽 틸트 2.0°)

### 2) 온보드 제어 PC 사양 (On-board Computer Specifications)

#### 🖥️ 하드웨어 사양 (Hardware Specs)
| 구분 | 상세 사양 | 비고 |
| :--- | :--- | :--- |
| **디바이스 호스트명** | `cgv-omor1v2-01` | 로봇 탑재 온보드 PC |
| **프로세서 (CPU)** | **Intel® Core™ Ultra 7 155H** (Meteor Lake) | 16코어 / 22스레드 (6P + 8E + 2LPE), 최대 4.80 GHz, 24MB L3 캐시 |
| **AI 가속기 (NPU)** | **Intel® AI Boost NPU** (Device `7d1d`) | 온디바이스 AI NPU 가속 지원 |
| **메모리 (RAM)** | **32 GB** (가용 약 26 GiB) + Swap 2.0 GiB | 멀티 노드 및 AI 추론 여유 공간 확보 |
| **저장 장치 (Storage)** | **256 GB NVMe SSD** (WD PC SN740 NVMe 256GB, PCIe Gen4) | 가용 여유 공간 ~181 GB (사용률 19%) |
| **그래픽 (GPU)** | **Intel® Arc™ Graphics** (Integrated GT2, Device `7d55`) | 8 Xe-Cores |
| **네트워크 (Network)** | • Wi-Fi: Intel Wi-Fi 6E/7 (`wlo1`)<br>• Ethernet: Intel 2.5GbE LAN (`enp86s0`)<br>• VPN: NetBird Mesh VPN (`wt0`, IP: `100.96.194.210`) | 원격 제어 및 원격 모니터링 연동 |
| **주변기기 인터페이스** | • USB-UART Bridge: Silicon Labs CP210x (`10c4:ea60`, MCU/모터 제어)<br>• USB Webcam: Realtek HD Camera (`0bda:5856`) | udev rules 자동 심볼릭 링크 지원 |

#### 🐧 소프트웨어 및 런타임 환경 (Software Specs)
| 구분 | 세부 환경 | 상세 버전 / 설명 |
| :--- | :--- | :--- |
| **운영체제 (OS)** | Ubuntu Linux | **Ubuntu 22.04.5 LTS** (Jammy Jellyfish, 64-bit x86_64) |
| **커널 (Kernel)** | Linux Kernel | `6.8.0-124-generic` |
| **로보틱스 (Middleware)** | ROS 2 | **ROS 2 Humble Hawksbill** (`/opt/ros/humble`) |
| **빌드 도구** | C++ / ROS 빌드 시스템 | `colcon` (0.21+), `cmake` (3.22.1), `gcc/g++` (11.4.0), `make` (4.3) |
| **Python 환경** | Python 3 | **Python 3.10.12** (`pip`, `venv` 가상환경 지원) |
| **AI / 딥러닝 가속** | Vision & Deep Learning | • **OpenVINO**: `2026.3.1` (YOLO11 FP16 IR 가속, **~78.4 FPS / 12.8 ms**)<br>• PyTorch: `2.11.0+cpu` / Torchvision: `0.26.0+cpu`<br>• Ultralytics: `8.4.47` (YOLOv11n-seg)<br>• OpenCV: `4.9.0` / NumPy: `1.26.4` |
| **개발 및 유틸리티 도구** | CLI & 모니터링 | `git` (2.34.1), `gh` (GitHub CLI), `tmux` (3.2a), `htop` (3.0.5), `tree`, `jq`, `net-tools`, `ripgrep`, `fzf`, `ffmpeg` |
| **원격 접속 환경** | Remote Access | • NetBird VPN IP: `100.96.194.210` (`cgv-omo-01.nb.hsl.ee`)<br>• OpenSSH Server (포트 22)<br>• Sunshine / Moonlight 원격 데스크톱 지원 |

### 3) 시스템 자체 진단 및 가속 엔진 검증 (Diagnostics & Benchmark)
온보드 PC 환경 및 카메라, 시리얼, LUT, OpenVINO 추론 벤치마크를 원클릭으로 검증할 수 있습니다:
```bash
python3 verify_robot_env.py
```

### 4) 필수 데이터 파일 위치
```text
/home/cgv/data/
├── cam/
│   └── usb_webcam.yaml          # 카메라 인트린식 캘리브레이션 (fx=517, fy=513, 640x480)
├── vocab/
│   └── orb_vocab.fbow           # VSLAM용 ORB Vocabulary 바이너리 (44.9 MB)
├── db/
│   └── NTH4F.msg                # 뉴턴홀 4층 사전 구축 VSLAM Map DB (95.4 MB)
└── fsd/ (또는 src/freespace_detection/scripts/)
    ├── best.pt                  # 미세조정된 YOLOv11n-seg PyTorch 모델 가중치
    ├── best_openvino_model/     # Intel CPU/Arc iGPU 최적화 FP16 OpenVINO 엔진 (78.4 FPS)
    ├── col_to_ch_lut.npy        # 가로 픽셀 -> 각도 채널 변환 테이블
    ├── distance_lut.npy         # 세로 픽셀 -> 지면 거리 변환 테이블
    └── distance_lut_2d.npy      # 2D 유클리드 기하 보정 지면 거리 변환 테이블
```

---

## 🔨 4. 빌드 방법 (Build Instructions)

```bash
# 1. 워크스페이스 디렉터리 이동
cd ~/ros2_ws

# 2. ROS 2 환경 로드
source /opt/ros/humble/setup.bash

# 3. 전체 패키지 빌드
colcon build --symlink-install --allow-overriding image_tools

# 4. 워크스페이스 환경 반영
source install/setup.bash
```

### 시리얼 포트 권한 설정 (최초 1회)
```bash
sudo ./udev_rules.sh
```

---

## 🚀 5. 실험 및 실행 가이드 (Experiment Guide)

### 🔹 [실험 A] 가상 라이다(V-LiDAR) 단독 센서 검증 실험
로봇을 주행시키지 않고, 카메라 앞에 장애물을 배치했을 때 가상 2D LaserScan이 정상적으로 생성되는지 확인하는 정적 거리 추정 실험입니다.

```bash
# 터미널 1: 카메라 및 V-LiDAR 노드 일괄 실행
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch robot_launch_package multi_node_launch.py

# 터미널 2: 실시간 141채널 스캔 값 확인
source /opt/ros/humble/setup.bash
python3 ~/ros2_ws/src/freespace_detection/scripts/scan_viewer.py
```
- **검증 내용**: 카메라 전방 1.0m ~ 3.0m 거리에 상자를 배치했을 때, `scan_viewer`에 출력되는 `valid min/max/mean` 거리 값이 실제 물리적 거리와 오차 0.26m 이내로 일치하는지 확인합니다.

---

### 🔹 [실험 B] 전체 자율주행 및 장애물 회피 실험 (논문 실험 재현)
Visual SLAM과 Nav2, V-LiDAR를 모두 연동하여 복도 환경에서 정적 장애물을 실시간으로 우회하는 통합 실험입니다.

```bash
cd ~/ros2_ws
./run_ros2_launch.sh
```

1. **실행 흐름**:
   - 백그라운드로 `multi_node_launch.py`(센서, 모터 제어, 가상 라이다, Nav2, RViz)가 구동됩니다.
   - 새 터미널 창에서 `stella_vslam_ros run_slam`이 실행되며 카메라 영상의 특징점(Feature) 매칭을 통해 로봇의 위치(Tracking)를 잡습니다.
2. **목표 지점 전달**:
   - RViz 화면 상단의 **`2D Goal Pose`** 버튼을 클릭하여 로봇 전방 10m 지점(복도 끝)을 클릭합니다.
3. **결과 관찰**:
   - 경로 중앙에 배치된 박스 장애물을 V-LiDAR가 감지하여 Local Costmap에 장애물 레이어(노란색/붉은색 인플레이션)를 생성합니다.
   - DWB Local Planner가 장애물을 안전하게 우회하는 회피 경로를 생성하여 주행을 완료합니다.

---

### 🔹 [실험 C] 키보드 수동 원격 조종 (Teleoperation)
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run omo_r1_teleop teleop_keyboard
```

---

### 🔹 [실험 D] 자율주행 주행 로그 수집 및 평가 리포트 자동 생성
주행 실험 시 백그라운드에서 실시간 이동 궤적, 속도, VSLAM 추적 상태, 최소 장애물 거리를 5Hz로 기록하고 종합 리포트(`summary_report.md`)를 생성합니다:
```bash
# 기본 실행 (맵: NTH4F)
python3 experiments/run_experiment.py --scenario corridor_box_avoidance_trial1

# 실험 종료(Ctrl+C) 시 experiments/logs/ 하위에 metadata.json, driving_log.csv, summary_report.md 자동 저장
```

---

### 🔹 [실험 E] Freespace Detection 바닥 세그멘테이션 정밀 벤치마크
로봇을 주행시키지 않고도, 다양한 조명/바닥 환경에서 YOLO 바닥 분할의 실시간 FPS, 지연 시간, 바닥 인식 면적, V-LiDAR 3구역(좌/중/우) 거리 품질을 정량 측정합니다:
```bash
python3 experiments/evaluate_freespace.py --duration 15 --scenario glossy_floor_eval
# 결과 리포트 및 프레임별 CSV가 experiments/logs/ 에 저장됩니다.
```

---

## 📚 6. 학술 인용 (Citation)

본 코드를 연구에 활용하실 경우 아래 논문을 인용해 주시기 바랍니다:

```bibtex
@inproceedings{lee2025vision,
  title={Vision-Based 2D Scan Generation for Obstacle Avoidance Using Floor Segmentation},
  author={Lee, Hyunseo and Yoo, Gunmin and Gu, Hyunwoo and Hwang, Sung Soo},
  booktitle={Extended Abstract},
  year={2025},
  organization={Handong Global University}
}
```
