# [교수님 보고용] IEEE Access 심사 요약 및 개선 가능 범위 (`Access-2026-51740`)

- **대상 논문**: *V-LiDAR: Real-Time Monocular Floor Segmentation and Virtual 2D Scan Synthesis for Autonomous Mobile Robot Obstacle Avoidance*
- **심사 결과**: **Reject (Do not accept resubmissions)** — 판정일 `2026-11-12` (AE: Dr. Kumaradevan Punithakumar)
- **상세 근거 문서**: [리뷰 원문(`01`)](./01_IEEE_Access_Decision_and_Raw_Reviews.md) · [사전 분석(`02`)](./02_Review_Analysis_and_Asset_Audit_Report.md) · [실행 플랜(`03`)](./03_Revision_and_Experiment_Action_Plan.md) · [로봇 PC 실물 감사(`04`)](./04_Robot_PC_Asset_Audit_and_Log_Inventory.md)

---

## 1. 심사 결과 핵심 요약

- **판정 요약**: 3인의 리뷰어 모두 **원고의 체계성·가독성(R2 Q3, R1 체크리스트 4개 전원 `Yes`)과 참고문헌 적절성(전원 부적절 문헌 `No`)은 긍정적으로 평가**했습니다. 다만 **물리 LiDAR 비교, 환경·거리 다양화, 학습 데이터 명세 등 신규 실험/검증 요구가 커서** IEEE Access 운영 방침상 단기 수정 대신 `Reject (재투고 불가)`로 결정되었습니다.
- **투고 직전 수정 효과**: Table 6 SIL Offline Replay 투명화, MDE affine 스케일 보정, 과도한 Claim 완화, AI 활용 공시 반영으로 형식적·윤리적 결함 지적은 피했습니다.

### 리뷰어 3인 평가 및 핵심 지적

| 리뷰어 | 체크리스트 / 평가 기조 | 긍정 평가 | 핵심 보완 요구 |
| :--- | :--- | :--- | :--- |
| **Reviewer 1** | **4개 항목 전부 `Yes`** (부적절 문헌 `No`) | 연구 기여·기술적 타당성·서술 포괄성 모두 인정 | ① 카메라 높이($h$)·틸트($\gamma$) 변경 실험<br>② 경량 실시간 MDE(`FastDepth`, `Lite-Mono`)·반사 오탐 지표 비교<br>③ 반사바닥·유리벽·저조도·동적 보행자 테스트<br>*(비고: Comment #3에 본 연구와 무관한 PatchTST 등 3편 인용 제안 포함)* |
| **Reviewer 2** | **모티베이션·안전성 정면 비판** | 논문이 체계적이고 잘 작성·정리됨 | ① **$2.18\text{ m}$ 사각지대 및 초음파/ToF 보완 권고가 '카메라 단독 LiDAR 대체' 주장을 약화**<br>② 학습·라벨링·캘리브레이션 등 **S/W 총소유비용(TCO)** 누락<br>③ 고사양 PC(`Core Ultra 7 155H`) 외 저사양 임베디드 보드 검증 부재<br>④ EMA/Persistence **동적 지연** 및 신뢰도(`c_conf`) **불확실성/OOD** 부재<br>⑤ 기존 Virtual LiDAR·카메라 회피 선행연구 비교 부족 |
| **Reviewer 3** | **가장 건설적 (5대 필수 보완 제시)** | 문제 실용성, 구현 난이도, 솔직한 실패(`run08/12`) 분석 호평; *"ROS 2 Nav2 통합 시스템 엔지니어링 기여"*로 규정 | ① **동일 로봇 물리 2D LiDAR 비교** (`RPLIDAR/YDLIDAR`)<br>② **YOLOv11n-seg 데이터셋·분할·학습·정확도(mAP/mIoU) 공개**<br>③ `2.50 m` 단일 거리/홀 $\rightarrow$ **다중 거리(`2.2–6.0 m`)·다양한 환경·동적 장애물 확장**<br>④ **`run08/12` Nav2 수정 설정 재주행(Before/After)**<br>⑤ **Operating Envelope(`2.2–6.0 m`, 밝은 개방 실내) 명시 및 톤다운** |

### 3인 공통 지적
1. **실험 공간·거리·장애물 다양성 부족** (`R1 #4`, `R2 #8·#10`, `R3 #3`): `2.50 m` 정지 측정 1지점(25프레임) + 넓은 홀 10 m 직진(박스 1개)에 국한됨.
2. **비교 Baseline 타당성 부족** (`R1 #2`, `R2 #4`, `R3 #1`): 동일 로봇 물리 2D LiDAR 비교 0건 및 경량 MDE/Virtual LiDAR 비교 부재.
3. **'LiDAR 대체' 포지셔닝 과장 vs $2.18\text{ m}$ 사각지대** (`R1 #1`, `R2 #1·#9`, `R3 #5`): 적용 한계 대비 주장 범위가 과대함.

---

## 2. 개선 가능 범위 구분 (로봇 PC 실물 자산 감사 기반)

로봇 PC(`/home/cgv`) 내 코드·가중치·rosbag 전수 조사 결과([`04` 보고서](./04_Robot_PC_Asset_Audit_and_Log_Inventory.md)), **추가 주행 없이 개선 가능한 항목(A)**과 **실차/물리 센서가 필수인 항목(B)**으로 명확히 나뉩니다.

### A. 추가 실차 주행 없이 바로 / 오프라인으로 개선 가능한 부분

| 항목 | 해당 지적 | PC 보유 근거 및 실행 내용 |
| :--- | :---: | :--- |
| **1. 논문 스코프·TCO·동적 지연 수식 보강** | `R2 #1·#2·#5·#9`, `R3 #5` | - `'LiDAR 대체'` $\rightarrow$ **`'개방형·조명 확보 실내(2.2–6.0 m) ROS 2 Nav2 통합 계측/회피 시스템'`**으로 포지셔닝 조정<br>- S/W 비용(라벨링·학습·캘리브레이션) 공정 비교(TCO) 추가<br>- `10 Hz` 기준 $N_{\mathrm{onset}}=2$ (`200 ms`) + EMA($\alpha=0.6$) 동적 탐지 지연 정량화 |
| **2. YOLO 학습/검증 지표 보강** | `R3 #2` *(부분)* | - [`best.pt`](../../src/freespace_detection/scripts/best.pt)에서 추출한 [`yolo11n_seg_train_results.csv`](../../experiments/logs/pc_extracted/yolo11n_seg_train_results.csv)로 **100 에포크 곡선·하이퍼파라미터(`batch=16`, `imgsz=320`, `lr0=0.01`)·Mask 정확도(`mAP50=0.80222`, `mAP50-95=0.56231`, `P=0.80468`, `R=0.76136`)** 즉시 반영 가능<br>- *(단, 원본 이미지 장수·train/val/test 분할만 학습 PC/Roboflow 기록에서 추가 확인 필요)* |
| **3. `run08`·`run12` 실패 원인 입증 강화** | 실패 분석 보강 | - [`minimal_A_scen1_20runs_metrics.csv`](../../experiments/logs/pc_extracted/minimal_A_scen1_20runs_metrics.csv) 상 두 실패 주행 모두 주행 중 `/floor_detector` **`100.0%` True**(`1016/1016`, `689/689`) 유지 확인 $\rightarrow$ 바닥 인식 실패가 아니라 Nav2 costmap 교착임을 직접 뒷받침 가능 |
| **4. PC 잔존 40개 Bag(`30,706프레임`) 기반 오프라인 실험** | `R1 #2`, `R2 #6·#7`, `R1 #4·R2 #10` *(일부)* | - Active(`28개`) + 휴지통(`12개`)에 남은 **회전의자(`nav_debug_avoidance`), 금속 카트+계단실 광원(`vlidar_run01.3`), 창문 역광+박스(`vlidar_run01.4`), 좁은 복도 정수기·점자블록+출입구 글레어(`vlidar_run01.5`, `floor_true=7.74%`)** 프레임 재사용 가능:<br>  ① 경량 MDE(`FastDepth`, `Lite-Mono` 등) 비교 확장 + [`benchmark_depth_baselines.py`](../../experiments/benchmark_depth_baselines.py) **yaw 미보정 오차 버그 수정** 재평가<br>  ② `c_conf` 민감도 / [`usb_webcam_calibration.yaml`](../../experiments/logs/pc_extracted/usb_webcam_calibration.yaml) 기반 `cv2.undistort` 유무 비교 / CPU 코어 제한 지연 시뮬레이션 |

### B. 로봇 실차 가동 / 물리 센서가 반드시 필요한 부분

| 필수 보완 항목 | 해당 지적 | PC 감사상 불가 사유 및 필요 조치 |
| :--- | :---: | :--- |
| **1. 동일 로봇 물리 2D LiDAR 동시 비교** | **`R3 #1`** *(핵심)* | - 현재 로봇 PC에 `/dev/ttyLiDAR` 장치 및 `ydlidar_ros2_driver`가 없어 기존 로그만으로 대체 불가<br>- 물리 LiDAR(`YDLIDAR`/`RPLIDAR`) 장착 시 `/scan_lidar` 동시 녹화 1~2일이면 `R3 #1` 1:1 비교와 `R3 #3` 다중거리 GT를 동시에 확보 가능 |
| **2. 수정 Nav2 파라미터 10회 실차 재주행** | **`R3 #4`** | - [`omo_r1.yaml`](../../src/omo_r1_navigation2/param/omo_r1.yaml) 최신 설정(`use_sim_time: False`, `inflation_radius: 0.75`, `cost_scaling_factor: 2.7`, `BaseObstacle.scale: 5.0`)으로 **10 m 박스 코스 재주행 Before/After 비교표** 필요 |
| **3. 유리벽·저조도·보행자·카메라 틸트($\gamma$) 실측** | **`R1 #1·#4`**, **`R3 #3`** | - PC의 360° 영상(`OH3F.mp4`)은 투영 모델이 달라 핀홀 LUT 정량 GT로 쓸 수 없어 스팟 테스트 필요 |
