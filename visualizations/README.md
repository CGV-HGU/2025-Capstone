# 📊 V-LiDAR 자율주행 팩트 기반 시각화 대시보드 (Visualization Index)

본 디렉터리는 로봇 워크스페이스 내의 실제 소스코드, 맵 파일, VSLAM 키프레임 궤적, Nav2 파라미터의 **실측 수치(Ground Truth)만을 기반으로 제작된 정밀 시각화 자료 모음**입니다.

---

## 📁 3대 핵심 시각화 분류

```text
visualizations/
├── README.md                                         # 본 마스터 인덱스 문서
│
├── 01_Maps_and_Coordinates/                          # 🗺️ 1. 주행 맵 및 글로벌 좌표계
│   ├── 01_NTH4F_Nav2_2D_Occupancy_Grid_Map.png       # 뉴턴홀 4층 2D 격자 및 실측 미터 좌표
│   ├── 02_OH3F_Visual_SLAM_Trajectory_Reconstruction.png # 오석관 3층 VSLAM 키프레임 2D 궤적 복원
│   ├── 03_Map_Resolution_and_Scale_Evolution.png     # 6개 2D 맵 변천사 및 해상도 비교
│   └── README.md                                     # 맵 상세 규격 및 원점 설명서
│
├── 02_System_and_Nav2_Dataflow/                      # ⚡ 2. 시스템 및 Nav2 연동 구조
│   ├── 01_V-LiDAR_to_Nav2_Complete_Pipeline.png      # 카메라 ➔ AI ➔ V-LiDAR ➔ Nav2 데이터 파이프라인
│   ├── 02_Nav2_Costmap_Marking_and_Raytracing.png    # 코스트맵 장애물 마킹(3m) & 클리어링(3.5m) 반경
│   └── README.md                                     # ROS 2 토픽 및 TF 관계 상세 설명서
│
└── 03_Sensor_and_AI_Benchmark/                       # 🎯 3. AI 추론 및 V-LiDAR 센서 성능
    ├── 01_YOLO11_Inference_Latency_and_FPS_Benchmark.png # OpenVINO (78.4 FPS) vs PyTorch 실측 벤치마크
    ├── 02_V-LiDAR_141Ch_Sector_Distribution.png      # 141채널 3구역 (좌/중/우) 각도 분할도
    └── README.md                                     # 추론 레이턴시 및 3구역 거리 판정 설명서
```

---

## 🚀 빠른 바로가기 가이드
* **뉴턴홀 및 오석관 맵의 실측 크기와 좌표가 궁금할 때**: 👉 [`01_Maps_and_Coordinates/README.md`](01_Maps_and_Coordinates/README.md)
* **V-LiDAR가 Nav2와 어떻게 통신하고 장애물을 지우는지 볼 때**: 👉 [`02_System_and_Nav2_Dataflow/README.md`](02_System_and_Nav2_Dataflow/README.md)
* **YOLO11 모델의 실제 추론 FPS와 화각 분할이 궁금할 때**: 👉 [`03_Sensor_and_AI_Benchmark/README.md`](03_Sensor_and_AI_Benchmark/README.md)
