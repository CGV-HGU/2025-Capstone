# 🎯 03. 센서 및 AI 추론 성능 벤치마크 (Sensor & AI Benchmark)

본 디렉터리는 온보드 PC(Intel Core Ultra 7 155H)에서 실측된 **YOLO11 바닥 세그멘테이션 추론 속도**와 **141개 채널 가상 라이다(V-LiDAR) 3구역 분기 구조**를 시각화한 자료입니다.

---

## 📊 시각화 자료 목록

### 1. `01_YOLO11_Inference_Latency_and_FPS_Benchmark.png`
* **추론 엔진 비교 ([`verify_robot_env.py`](file:///home/cgv/ros2_ws/verify_robot_env.py) 실측치)**:
  * **OpenVINO FP16 Engine**: **78.4 FPS** (평균 **12.8 ms**)
  * **PyTorch CPU Vanilla**: **59.8 FPS** (평균 **16.7 ms**)
  * **카메라 입력 기준선**: 30 FPS (데드라인 33.3ms)
  * **백분위수 레이턴시**:
    * P50 (중앙값): OpenVINO 12.5ms vs PyTorch 16.2ms
    * P95: OpenVINO 15.4ms vs PyTorch 21.0ms
    * P99: OpenVINO 18.2ms vs PyTorch 25.5ms

### 2. `02_V-LiDAR_141Ch_Sector_Distribution.png`
* **141채널 각도 분할 (화각 $\pm 35^\circ$, 채널당 $0.5^\circ$)**:
  * **Left Sector** (채널 0 ~ 46, $-35^\circ \sim -12^\circ$): 좌측 벽면 및 장애물 감지
  * **Center Sector** (채널 47 ~ 93, $-11^\circ \sim +11^\circ$): **정면 충돌 위험 감시 구역 (최우선)**
  * **Right Sector** (채널 94 ~ 140, $+12^\circ \sim +35^\circ$): 우측 벽면 및 장애물 감지
