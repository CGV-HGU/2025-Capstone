# V-LiDAR 후속 점검, 보완 실험 및 차기 저널 재투고 실행 플랜

본 문서는 IEEE Access(`Access-2026-51740`) 심사 결과를 바탕으로 **① 로봇 PC 접속 시 사전 점검 절차**, **② 무실험(Track A) 및 소규모 실차(Track B) 보완 플랜**, **③ 차기 타겟 저널 선정 및 원고 개정 전략**을 실행 순서대로 정리한 가이드입니다.

---

## Phase 0. 로봇 PC(`cgv@100.96.194.210`) 접속 후 1차 현장 점검

로봇 온보드 PC(Ubuntu 22.04 / ROS 2 Humble, `~/ros2_ws`)에 SSH 접속 후 아래 명령어를 실행하여 재활용 가능한 데이터와 하드웨어 가용 여부를 확인합니다. (NetBird VPN 연결 필요)

```bash
# 0-1. 브랜치 동기화
cd ~/ros2_ws
git fetch origin && git checkout post && git pull origin post

# 0-2. 기존 Rosbag 보관 상태 및 토픽 구성 확인 (Track A & B 판단 기준)
ls -lh ~/data/bags/
# vlidar_run01 등 주요 bag 내부 토픽 목록 확인 예시:
ros2 bag info ~/data/bags/vlidar_run01 2>/dev/null

# 0-3. 학습 원본 데이터셋 / Roboflow / YOLO runs 디렉토리 탐색 (R3 #2 완결용)
find ~ -maxdepth 4 \( -name "data.yaml" -o -name "results.csv" -o -name "args.yaml" \) 2>/dev/null

# 0-4. 물리 2D LiDAR (YDLIDAR 등) 포트 및 드라이버 확인 (R3 #1 수행 가능 여부)
ls -l /dev/ttyLiDAR /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
ros2 pkg list | grep -E "ydlidar|rplidar|urg"

# 0-5. 현재 Nav2 파라미터 배포 상태 확인 (R3 #4용)
grep -nE "inflation_radius|BaseObstacle|use_sim_time" ~/ros2_ws/src/omo_r1_navigation2/param/omo_r1.yaml
```

### 점검 후 의사결정 분기
- **물리 LiDAR 가용 + 로봇 1~2일 주행 가능** $\rightarrow$ **Track A + Track B 전부 수행** 후 **IOP *Measurement Science and Technology* (SCIE Q2)** 투고 (가장 권장).
- **로봇 실차 가동 불가(졸업/일정 종료), 기존 Rosbag만 존재** $\rightarrow$ **Track A 오프라인 보강 + 포지셔닝 전면 하향** 후 타겟 저널 재조정(교수님 협의).

---

## Phase 1. [Track A] 추가 로봇 주행 없이 즉시 실행 가능한 보완 작업

로봇을 움직이지 않고 기존 코드, 가중치(`best.pt`), 기존 rosbag(`~/data/bags/`), 원고 수정만으로 해결하는 작업입니다.

### A-1. YOLOv11n-seg 학습/검증 프로토콜 완전 기재 (`R3 #2` 해결)
- **이미 확보된 수치** ([`best.pt`](../../src/freespace_detection/scripts/best.pt)): `epochs=100`, `batch=16`, `imgsz=320`, `lr0=0.01`, `mosaic=1.0`, `randaugment`, Mask `mAP50=0.802`, `mAP50-95=0.562`, Precision `0.805`, Recall `0.761`.
- **추가 확인 사항**: Phase 0-3에서 찾은 데이터셋의 총 이미지 장수, 출처(직접 촬영/공개셋), `train/val/test` 분할 수, 라벨링 툴/기준을 Section III-B에 소표(Table) 또는 문단으로 추가.

### A-2. 경량 MDE 오프라인 비교 확장 (`R1 #2` 해결)
- 대상 스크립트: [`experiments/benchmark_depth_baselines.py`](../../experiments/benchmark_depth_baselines.py), [`experiments/run_dual_bag_benchmark.py`](../../experiments/run_dual_bag_benchmark.py)
- 로봇 PC에 남은 `vlidar_run01`, `vlidar_run02` rosbag을 사용해 **경량 실시간 MDE(`FastDepth`, `Lite-Mono`)** 1~2종을 추가 실행하고, Table 5에 `Latency`, `FPS`, `Params`, `Dynamic MAE`, `False Obstacle Rate`를 보강.
- Related Work 및 비교표에 기존 **Virtual LiDAR / Image-to-LaserScan / Monocular Traversability** 선행연구 3~5편 추가 인용 (`R2 #4`, `R2 Q4` 해결).

### A-3. Temporal Filter 동적 지연 및 Confidence 불확실성 정량 분석 (`R2 #5`, `R2 #7`)
- **동적 지연 정량화 (`R2 #5`)**: `10 Hz` (`100 ms`/frame) 기준 $N_{\mathrm{onset}}=2$는 최소 **`200 ms`의 onset 탐지 지연**을 유발하며, $\alpha=0.6$ EMA가 90% 수렴하는 데 걸리는 프레임 수에 따른 보행자 속도($0.5\text{ m/s}, 1.0\text{ m/s}$)별 예상 거리 지연을 Section III-F / Discussion에 수식으로 정직하게 분석.
- **불확실성 분석 (`R2 #7`)**: 기존 rosbag 프레임을 재실행해 `c_conf` 임계값($0.15, 0.25, 0.40$) 변화에 따른 거리 MAE / False Obstacle Rate 트레이드오프 곡선 산출 및 OOD(미학습 바닥) 안전 한계 명시.

### A-4. 논문 프레이밍 및 TCO 재정비 (`R2 #1·#2`, `R3 #5·Q1`)
- **핵심 포지셔닝 전환**:
  - *(기존)* "물리적 LiDAR 없는 저비용 자율주행 대안"
  - *(개정)* **"개방형·조명 확보 실내 환경($2.2\text{--}6.0\text{ m}$ 정적 장애물, $v \le 0.3\text{ m/s}$)을 대상으로 한 ROS 2 Nav2 통합 및 단안 가상 스캔 계측 시스템(System Integration & Monocular Metrology)"**
- **Abstract & Conclusion**: Operating Envelope(`2.2–6.0 m`, well-lit flat hall, no near-field `<2.18 m` guarantee)를 첫 문단/결론에 명시.
- **Software TCO 비교 추가 (`R2 #2`)**: Table 1(센서 비교표) 또는 Discussion에 하드웨어 BOM(`<$20` vs `$300+`) 대비 소프트웨어 복잡도(데이터 수집, 라벨링, 재학습, 외인자 캘리브레이션)를 공정하게 대조.

---

## Phase 2. [Track B] 로봇 1~2일 가동 시 가성비 극대화 실차 실험 (강력 추천)

90회씩 대규모 반복을 할 필요 없이, 리뷰어 3명이 공통 지목한 빈칸만 핀포인트로 메우는 **고효율 3대 실험**입니다.

### B-1. [최우선 1순위] 물리 2D LiDAR(YDLIDAR) 동시 Rosbag 로깅 (`R3 #1` + `R3 #3` + `R1 #4` 동시 해결)
- **목적**: V-LiDAR 카메라와 물리 2D LiDAR를 동일 OMO-R1에 동시 장착해, LiDAR를 **독립 Ground Truth(GT) 겸 1:1 비교 Baseline**으로 활용.
- **주의사항 (토픽 충돌 방지)**: [`fake_lidar_with_tf`](../../src/fake_lidar_with_tf/fake_lidar_with_tf/fake_lidar_with_tf.py#L30)와 [`omo_r1_lidar.launch.py`](../../src/omo_r1_bringup/launch/omo_r1_lidar.launch.py) 모두 기본 `/scan` 토픽을 사용하므로, 물리 LiDAR 실행 시 반드시 **`/scan:=/scan_lidar`**로 리매핑해야 함.
  ```bash
  # 터미널 A: V-LiDAR 기본 통합 실행
  ./start_all.sh

  # 터미널 B: 물리 YDLIDAR 별도 토픽(/scan_lidar)으로 동시 실행
  ros2 launch omo_r1_bringup omo_r1_lidar.launch.py --ros-args -r /scan:=/scan_lidar
  # (LifecycleNode 자동 activate 안 될 경우 별도 확인: ros2 lifecycle set /ydlidar_ros2_driver_node configure && activate)

  # 터미널 C: /scan(V-LiDAR), /scan_lidar(물리 LiDAR), /camera/image_raw, /odom 동시 녹화
  ros2 bag record -o ~/data/bags/dual_lidar_comp_<시나리오> \
    /camera/image_raw /scan /scan_lidar /odom /tf /tf_static
  ```
- **필수 촬영 시나리오 (각 1~2회씩만 찍어도 압도적 근거 확보)**:
  1. **거리별 계층 정적 측정 (`R3 #3`)**: 장애물을 $2.2\text{ m}$(사각지대 직전), $2.5\text{ m}$, $3.5\text{ m}$, $4.5\text{ m}$, $5.5\text{ m}$로 옮기며 각 10초 정지 녹화 $\rightarrow$ `V-LiDAR vs 물리 LiDAR` 전 구간 거리 오차 곡선 생성.
  2. **악조건 4종 스팟 비교 (`R1 #4`, `R2 #10`, `R3 #3`)**: ① 광택 바닥 조명 반사, ② **투명 유리문/파티션**(물리 LiDAR가 뚫릴 때 V-LiDAR 반응 대조), ③ 저조도(형광등 절반 소등) 및 그림자, ④ 다른 바닥 재질(카펫/타일 경계 또는 복도) + 보행자 횡단/접근 클립.
  3. **동일 경로 10 m 폐루프 주행 (`R3 #1`)**: 물리 LiDAR `/scan_lidar`로 Nav2를 돌린 3~5회 주행 vs V-LiDAR `/scan`으로 돌린 주행의 경로 길이·시간·여유 거리 비교.

### B-2. 수정된 Nav2 파라미터 실차 재주행 (`R3 #4` 해결)
- 현재 [`omo_r1.yaml`](../../src/omo_r1_navigation2/param/omo_r1.yaml#L125-L157) 또는 논문 제안값(`inflation_radius: 0.55`, `BaseObstacle.scale: 2.5`, `use_sim_time: False`)으로 `colcon build` 후, **`run08`·`run12`와 동일한 10 m 박스 회피 코스를 10회 연속 재주행**.
- Before(`18/20, 90.0%`, 정지 2회) vs After(`예: 10/10, 100.0%`) 비교표로 `R3 #4` 완전 종결.

### B-3. 카메라 틸트 각도($\gamma$) 변경 정적 테스트 (`R1 #1` 해결)
- 카메라 마운트 틸트만 2가지(예: 기존 $\gamma=2.0^\circ$ vs 하향 $\gamma \approx 7.0^\circ$)로 변경해, 동일 박스·소형 장애물($h_{\mathrm{obs}} \approx 0.25\text{ m}$)을 $1.8\text{ m}, 2.0\text{ m}, 2.5\text{ m}$에 두고 $D_{\min}$ 감소($2.18\text{ m} \to 1.77\text{ m}$) 이론식과 실측 일치 여부 검증.

---

## Phase 3. 차기 투고 저널 전략 및 원고 이식

IEEE Access(`Access-2026-51740`)는 재투고가 영구 불가하므로 아래 대안 저널 중 택일합니다.

1. **1순위 추천: *Measurement Science and Technology* (IOP MST / MSAT, SCIE Q2, IF 2.7, 구독 모델 APC $0)**
   - **기존 워크스페이스**: [`paper/workspace/IOP_MST/`](../workspace/IOP_MST/README.md)
   - **적합 이유**: Reviewer 2와 같은 "카메라만으로 LiDAR를 완벽 대체하느냐"는 자율주행 시스템 리뷰어의 거부감을 피해, **"단안 비전 기반 평면 2D 거리 계측(Optical Metrology), 2D LUT 18% 외곽 왜곡 제거, $D_{\min}$ 광학 사각지대 경계 정식화, 물리 LiDAR 대비 유리/반사면 계측 특성 비교"**라는 **센서·계측 학술 가치**로 정면 승부 가능.
   - **사전 필수 작업**: 현재 `IOP_MST/main.tex`는 `2026-10-01` 버전이므로, 최신 `IEEE_Access/main.tex`의 개정 내용(SIL Replay 명시, MDE affine 정렬, 문헌 39편, AI 공시)을 먼저 이식하고 Phase 1~2 데이터를 반영할 것.
2. **2순위 대안: *Computers & Electrical Engineering* (Elsevier C&EE, SCIE)**
   - **기존 워크스페이스**: [`docs/papers/Elsevier_CEE/`](../../docs/papers/Elsevier_CEE/cover_letter.txt) / `paper/workspace/Elsevier_CEE/`
   - **적합 이유**: OpenVINO FP16 엣지 가속, LUT 연산 최적화, ROS 2 Nav2 미들웨어 통합 관점에 부합. 단, R2 #6(임베디드 연산 제약 검증: 예-코어/클럭 제한 테스트 또는 RPi/Jetson 측정)이 보강되면 방어력이 크게 상승함.

---

## 부록. 리뷰어 전체 지적사항-대응 매핑표 (Traceability Matrix)

| 리뷰어 항목 | 핵심 요구 요약 | 분류 | 대응 작업 / 섹션 |
| :--- | :--- | :---: | :--- |
| **R1 #1** | 카메라 $h, \gamma$ 변경 체계적 테스트 & 파라미터 선택 전략 | **Track B** | Exp B-3 + Sec. IV-C 실측표 추가 |
| **R1 #2** | 경량 MDE(`FastDepth`, `Lite-Mono`) 및 반사 오탐률/오차 비교 | **Track A** | Exp A-2 + Table 5 확장 |
| **R1 #3** | 무관 논문 3편(PatchTST, YOLO-TCS, hand heat trace) 언급 | **제외** | 주제 무관 인용 강요로 판단 — 차기 투고 시 반영 불필요 |
| **R1 #4** | 반사 바닥, 유리 파티션, 저조도, 동적 보행자 테스트 | **Track B** | Exp B-1 시나리오 (2) 스팟 정량 평가 추가 |
| **R2 #1, #9** | 2.18 m 사각지대 + 초음파 추가 언급에 따른 모티베이션 약화 | **Track A** | Task A-4 (계측/통합 프레임 전환, Operating Envelope 명시) |
| **R2 #2** | 딥러닝 학습·캘리브레이션 등 소프트웨어 TCO 분석 누락 | **Track A** | Task A-4 (Table 1/Discussion에 S/W 복잡도 비교 추가) |
| **R2 #3, #8, #10** | 다양성·미학습 바닥·조명·클러터 강건성 근거 부족 | **Track B** | Exp B-1 악조건 데이터 + 솔직한 도메인 한계 명시 |
| **R2 #4, Q4** | SOTA 카메라 장애물 회피 / Virtual LiDAR 선행연구 비교 | **Track A** | Task A-2 (Related Work 문헌 보강 및 Table 비교) |
| **R2 #5** | EMA/Persistence 필터가 동적 장애물 탐지 지연에 미치는 영향 | **Track A** | Task A-3 (`200 ms` lag + 속도별 지연 수식/표 추가) |
| **R2 #6** | Core Ultra 7 외 저사양 임베디드 보드 성능 검증 부재 | **Track B(선택)** | 저전력 코어 제한(`taskset`/전력모드) 또는 별도 보드 측정 / 한계 명시 |
| **R2 #7** | 신뢰도 Calibration, 불확실성 전파, OOD 안전 Fallback 부재 | **Track A** | Task A-3 (`c_conf` 민감도 곡선 + OOD 한계·안전 전략 서술) |
| **R3 #1** | 동일 로봇 물리 2D LiDAR(RPLIDAR/YDLIDAR) 비교 | **Track B** | Exp B-1 (동시 `rosbag` 비교) |
| **R3 #2** | YOLO 데이터셋·분할·학습 하이퍼파라미터·세그멘테이션 정확도 | **Track A** | Task A-1 (`best.pt` 수치 + Phase 0-3 데이터셋 정보 기재) |
| **R3 #3** | 다중 거리(2.2–6.0 m), 다중 각도, 부분 가림, 동적 장애물 확장 | **Track B** | Exp B-1 거리 계층 정적/동적 로깅 |
| **R3 #4** | Nav2 파라미터 수정 후 전체 실차 재주행 Before/After 보고 | **Track B** | Exp B-2 (수정 설정으로 10회 이상 재주행 검증) |
| **R3 #5, Q1** | System Integration 수준으로 눈높이 조정 & Operating Envelope 명시 | **Track A** | Task A-4 (Abstract, Intro, Conclusion 전면 톤다운·명시) |
