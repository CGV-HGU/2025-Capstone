# 로봇 PC 실물 자산 감사 및 Git 푸시 가능 로그 인벤토리 (`cgv`, `2026-11-13`)

- **감사 대상**: 로봇 온보드/작업 PC (`/home/cgv`), `~/ros2_ws` (브랜치 `post`, 기준 커밋 `d440954` 직후), `/home/cgv/data/`, `/home/cgv/.local/share/Trash/files/`
- **연계 문서**: [`02_Review_Analysis_and_Asset_Audit_Report.md`](./02_Review_Analysis_and_Asset_Audit_Report.md) (Git 저장소 기준 사전 실사)의 실제 로봇 PC 현장 검증 결과
- **산출 로그 저장 위치**: [`experiments/logs/pc_extracted/`](../../experiments/logs/pc_extracted/README.md) 및 [`experiments/logs/`](../../experiments/logs/) (전체 약 `1.08 MiB`, `.gitignore` 화이트리스트 해제 완료)

---

## 요약: Git 저장소 기준 예측(`02`) vs 로봇 PC 실물 확인 결과

| 항목 | 사전 예상 (`02_Review_Analysis...`) | 로봇 PC 실물 확인 결과 (차이점 및 핵심 발견) |
| :--- | :--- | :--- |
| **`experiments/logs/`** | 로컬/Git에 `.gitkeep`만 존재 | 실제로는 **기존 벤치마크 로그 7개(`68 KB`)**가 존재했으나 `.gitignore`의 `/experiments/logs/*` 규칙에 가려져 Git에 푸시되지 않고 있었음 |
| **미보고/숨김 Rosbag** | 논문 `scen1` 20회 + `vlidar_run01..02`만 예상 | **`/home/cgv/data/bags/` 28개 + 휴지통(`Trash/files/`) 12개 = 총 40개 rosbag**, 카메라 프레임 총 **30,706장**(`active 3,586` + `trash 27,120`) 잔존 확인 |
| **다중 환경·장애물 영상** | 단일 홀 + 박스 1개만 존재할 것으로 추정 | 휴지통 bag에 **회전의자(`nav_debug_avoidance`)**, **금속 바퀴 카트+계단실 역광(`vlidar_run01.3`)**, **석양 창문 역광+박스(`vlidar_run01.4`)**, **좁은 복도 정수기+점자블록+출입구 포화(`vlidar_run01.5`)** 영상 다수 잔존 |
| **수정 후 Oct-2 주행** | `omo_r1.yaml` 수정 파라미터 주행 없음으로 추정 | `2026-10-02 19:15–19:20`에 **`minimal_run02`~`06` (연속 40 m 폐루프 주행 로그)** 실제 존재 (`minimal_run02`는 zstd 손상, `03`~`06` 정상). 단, 당시 `omo_r1.yaml` 최종 커밋 시점과의 정합성은 추가 대조 필요 |
| **물리 2D LiDAR 가용성** | YDLIDAR 드라이버·포트로 즉시 동시 로깅 가능 기대 | **포트(`/dev/ttyLiDAR`) 및 `ydlidar_ros2_driver` 패키지 미설치** (`/dev/ttyMCU -> ttyUSB0`, `/dev/ttyACM0`만 존재). 백업 `scan_echo.txt`도 `±0.6109 rad`(35°) V-LiDAR 출력이었음 |
| **YOLO 학습 데이터셋** | PC 또는 학습 서버에 `data.yaml`/`results.csv` 기대 | **원본 이미지/`data.yaml`/Roboflow 폴더는 PC 내 부재**. 대신 `best.pt` 내부에 **100 에포크 전체 학습/검증 곡선(`train_results`) 및 전체 하이퍼파라미터**가 완전 내장됨 |

---

## 1. 하드웨어, 환경, 캘리브레이션 및 모델 캐시 현황

### 1-1. 온보드 PC 스펙 및 설치 패키지
- **CPU / RAM**: `Intel(R) Core(TM) Ultra 7 155H` (16코어 22스레드), `30Gi RAM`
- **주요 Python/AI 스택**: `torch 2.11.0+cpu`, `ultralytics 8.4.47`, `openvino 2026.3.1`, `timm 1.0.30`, `transformers 5.18.0`, `cv2 4.9.0`, `scipy 1.8.0` (`onnxruntime`, `sklearn` 미설치)
- **오프라인 재사용 가능 MDE 모델 캐시**:
  - `/home/cgv/.cache/torch/hub/intel-isl_MiDaS_master`
  - `/home/cgv/.cache/torch/hub/fabio-sim_Depth-Anything-ONNX_main`
  - `/home/cgv/.cache/huggingface/hub`
- **ROS 2 / 센서 디바이스 상태**:
  - 연결 장치: `/dev/ttyMCU -> ttyUSB0` (`10c4:ea60`), `/dev/ttyACM0`, `/dev/video0`, `/dev/video1`
  - **부재 장치/패키지**: `/dev/ttyLiDAR` 장치 노드 없음, `/opt/ros/humble/share`에 `ydlidar_ros2_driver` 미설치 (`depthimage_to_laserscan`, `laser_geometry`만 존재).
  - `/home/cgv/backups/ros2_backup/scan_logs/scan_echo.txt` (`May 18`, `303,821 B`) 확인 결과 `angle_min: -0.610865`, `angle_max: 0.610865`로 물리 LiDAR가 아닌 **V-LiDAR `/scan` 기록**임. (별도 `/home/cgv/bs_ws`의 `ldlidar` 패키지는 UGV Beast 프로젝트용)

### 1-2. 카메라 캘리브레이션 및 모델 가중치
- **캘리브레이션 원본**: `/home/cgv/data/cam/usb_webcam.yaml` (`640x480`, `fx=517.1989`, `fy=513.1812`, `cx=318.9193`, `cy=252.4095`, `k1=0.03971`, `k2=0.13648`, `p1=-0.00111`, `p2=0.00048`, `k3=-0.92221`) $\rightarrow$ [`usb_webcam_calibration.yaml`](../../experiments/logs/pc_extracted/usb_webcam_calibration.yaml)로 복사 완료
- **`best.pt` 무결성**: 아래 3개 경로의 `best.pt` MD5(`4e8a79093d9ca3befa76fde5e4f84115`) 100% 동일:
  - `/home/cgv/ros2_ws/src/freespace_detection/scripts/best.pt`
  - `/home/cgv/data/fsd/best.pt`
  - `/home/cgv/Downloads/2025-Capstone-main/src/freespace_detection/scripts/best.pt`
- **내부 학습 정보 (`R3 #2` 핵심 보강 근거)**:
  - 학습 일시 `2024-10-24T23:37:44`, Ultralytics `8.3.4`, `yolo11n-seg.pt`, `epochs=100`, `batch=16`, `imgsz=320`, `lr0=0.01`, `names={0:'person', 1:'floor'}`
  - 최종 에포크(100): `metrics/precision(M)=0.80468`, `recall(M)=0.76136`, `mAP50(M)=0.80222`, `mAP50-95(M)=0.56231`, `mAP50(B)=0.80377` (최고 `mAP50(M)=0.81164` @ epoch 96, 최고 `mAP50-95(M)=0.56313` @ epoch 97)
  - 100 에포크 전체 곡선 $\rightarrow$ [`yolo11n_seg_train_results.csv`](../../experiments/logs/pc_extracted/yolo11n_seg_train_results.csv), 전체 인자 $\rightarrow$ [`yolo11n_seg_checkpoint_meta.json`](../../experiments/logs/pc_extracted/yolo11n_seg_checkpoint_meta.json)
- **OpenVINO 변환 모델**: `best_openvino_model/metadata.yaml` (`2026-09-30`, Ultralytics `8.4.47`, `imgsz=[256, 320]`, `half=true`) $\rightarrow$ [`openvino_model_metadata.yaml`](../../experiments/logs/pc_extracted/openvino_model_metadata.yaml)로 복사 완료

---

## 2. 미보고 / 휴지통 Rosbag 및 동영상 자산 상세 인벤토리

전체 40개 bag의 정량 수치는 [`rosbag_inventory_summary.csv`](../../experiments/logs/pc_extracted/rosbag_inventory_summary.csv)에 일괄 추출했습니다.

### 2-1. `/home/cgv/data/bags/` (28개, 총 `5.0 GB` + `rosbags_22_trials.tar.gz` `3.4 GB`)
1. **논문 Table III 보고 20회 (`minimal_A_scen1_run01`~`20`, `2026-09-21`)**
   - 별도 검증표: [`minimal_A_scen1_20runs_metrics.csv`](../../experiments/logs/pc_extracted/minimal_A_scen1_20runs_metrics.csv)
   - 성공 18회 평균: 총 녹화 `58.07 s`, 실주행(`odom_moving_dt_s`) `39.8 s`, 누적 경로 `10.195 m`
   - 정지 실패 2회 재확인:
     - `run08`: `121.249 s` (`16.138 s` 이동), `(3.7022, 0.8983)` 정지, `/floor_detector` **`100.0%` True** (1016/1016), 최소 스캔 `2.4128 m`
     - `run12`: `81.14 s` (`19.554 s` 이동), `(2.9144, 1.0691)` 정지, `/floor_detector` **`100.0%` True** (689/689), 최소 스캔 `2.2176 m`
   - 즉, 실패 2건은 바닥 분할 소실 때문이 아니라 Nav2 costmap/회복 동작 교착이었음이 `/floor_detector=100%` 로그로 직접 증명됨.
2. **논문 미보고 Active 주행 (`8개`)**
   - `minimal_A_scen2_run01` (`2026-09-23`): `68.37 s` (`39.95 s` 이동), `0 -> (9.776, 0.047)`, `max_abs_y=0.684 m`, `min_scan=2.363 m`
   - `minimal_run02` (`2026-10-02 19:16`): **`CORRUPT_ZSTD`** (`Read error (39) : premature end`로 복호 불가)
   - `minimal_run03`~`05` (`2026-10-02 19:17–19:19`): 연속 오도메트리 세그먼트 (`9.77->19.41`, `19.41->28.92`, `28.92->38.66`, 각각 `~9.8 m` 전진)
   - `minimal_run06` (`2026-10-02 19:20`): 최종 `0.31 m` 단거리 회전 (`12.60 s` 이동)
   - `vlidar_run01` (`2026-10-02 19:06`): `67.76 s`, `2,034` 카메라 프레임, 회전의자 장애물 회피 (`min_scan=2.178 m`)
   - `vlidar_run02` (`2026-10-02 19:30`): `53.86 s`, `1,552` 카메라 프레임, 회전 구간 포함 (`min_scan=2.178 m`)

### 2-2. `/home/cgv/.local/share/Trash/files/` (12개 bag, 카메라 **27,120장**)
| Trash 폴더명 | 크기 / 상태 | 프레임 | 주행 요약 | 컨택트시트 시각 검증 및 텔레메트리 핵심 |
| :--- | :---: | :---: | :--- | :--- |
| **`nav_debug_avoidance`** | `2.7G` (`db3`+`zstd`) | `1,971` | `65.67s` (`35.15s` 이동), `end=(9.83, 0.13)` | **유일하게 `/plan`(37), `/local_plan`(376), `/initialpose`(3) 포함**. 장애물은 **회전의자** |
| **`vlidar_run01`** | `409M` (`zstd`) | `803` | `28.13s`, `0.0s` 이동 | 정지 상태 사전 점검용 (`floor_true=100%`) |
| **`vlidar_run01.2`** | `696M` (`zstd`) | `1,411` | `47.17s` (`36.32s` 이동), `end=(9.77, 0.03)` | `max_abs_y=0.823m`, `floor_true=96.59%` |
| **`vlidar_run01.3`** | `2.8G` (무`yaml` `db3`) | `2,613` | `88.55s` (`64.69s` 이동), **`path=17.32m`**, `end=(9.91, 8.24)` | **금속 바퀴 카트(스탠드) 회피 + 후반 계단실 자연광 눈부심** (`floor_true=70.20%`) |
| **`vlidar_run01.4`** | `2.0G` (무`yaml` `db3`) | `2,305` | `76.91s` (`67.28s` 이동), **`path=19.72m`**, `max_x=9.12, end=(3.68, 8.55)` | **늦은 오후 창문 저각도 역광/바닥 반사 + 박스 장애물** (`floor_true=88.41%`) |
| **`vlidar_run01.5`** | `15G` (무`yaml` `db3`+`zstd`) | `15,895` | `582.17s` (`59.55s` 이동), `path=11.02m` | **넓은 홀 $\rightarrow$ 정수기·노란 점자블록이 있는 좁은 복도 + 출입문 백화(글레어)**. `/floor_detector` True 비율이 **`7.74%` (352/4548)**까지 급락한 귀중한 failure-mode 로그 |
| **`vlidar_run01.6`** | `1.1G` (`zstd`) | `2,122` | `70.73s` (`26.95s` 이동), `end=(8.21, -0.35)` | 복도 직진+완만한 회피 (`floor_true=100%`, `min_scan=2.439m`) |
| `minimal_A_scen1_run01/09`, `minimal_run02/02.2/03` | `0.5~1.2M` | `0` | 백업/선행 테스트 bag | `09-21 00:01` `run01`만 `max_vx=0.421 m/s`, 이후 전 주행 `0.3154 m/s` 상한 일치 |

### 2-3. 360° 원본 복도 영상 (`/home/cgv/data/db/video/`)
- `NTH4F.mp4` (`3.6G`, `3840x1920`, `510.91s`, `15,312f`), `OH3F.mp4` (`1.7G`, `3840x1920`, `240.61s`, `7,211f`, 오석관 3층 **유리벽 복도 및 기둥** 촬영).
- **한계**: 360° equirectangular 포맷이므로 `640x480` 핀홀 카메라 LUT에 직접 투입할 수 없으며, 원근 크롭 변환 시에도 V-LiDAR의 실제 장착 외인자(`h=1.05 m`, `tilt=2.0°`) 거리 GT로 사용하기 어렵습니다(R1 #4 유리벽 정량 평가 대체 불가).

---

## 3. 감사 과정에서 확인된 코드·데이터 기술적 주의사항 (중요)

1. **`benchmark_run02.csv` 및 `dynamic_trajectory_benchmark.csv`의 요(Yaw) 미보정 오차 버그**:
   - [`experiments/benchmark_depth_baselines.py`](../../experiments/benchmark_depth_baselines.py)가 동적 주행 중 로봇의 yaw 회전을 보정하지 않은 채 차체 정면 인덱스(`ranges[55:85]` / `scan_vl[60:81]`)만 잘라 GT 거리와 비교합니다.
   - 그 결과 `benchmark_run01.csv` (`n=31`, `x<3.8 & 2.18<=gt<=5.0`)에서는 V-LiDAR MAE가 `0.8058 m` (`[2.50, 3.00)` 구간 `0.3180 m`)로 양호하나, 회전이 큰 `benchmark_run02.csv` (`n=28`)에서는 `2.3303 m`, 합산 `baseline_benchmark_report.md`에서는 **`1.529 m`**(MiDaS `0.880 m`, DA-V2 `3.298 m`)로 왜곡됩니다.
   - **시사점**: 기존 `benchmark_run*.csv`를 논문 동적 MAE 표로 바로 인용하면 안 되며, 회전 전 직진 구간 또는 ROS TF/평면 투영 GT로 재평가 스크립트를 수정한 뒤 산출해야 합니다.
2. **`20260903_171652_eval_test_run` 주의**:
   - [`eval_report.md`](../../experiments/logs/20260903_171652_eval_test_run/eval_report.md)는 OpenVINO FP16 추론 지연(`mean 19.34 ms`, `P50 16.23 ms`, `P95 19.73 ms`, `51.7 FPS`)이 기록되어 있으나, 동시에 `평균 바닥 분할 면적 = 0.0% (±0.00%)`인 초기 테스트 산출물이므로 지연 벤치마크 보조 참고 외의 인식 품질 수치로는 사용 불가합니다.
3. **`floor_detector.py` 구현 불일치 2건**:
   - `inference_device='GPU'` 파라미터가 선언되어 있으나 실제로는 `best.pt` PyTorch CPU 경로만 로딩하고 해당 파라미터를 사용하지 않습니다(OpenVINO FP16는 별도 벤치마크/추출 메타 경로).
   - `/home/cgv/data/cam/usb_webcam.yaml`에 큰 방사 왜곡 계수(`k3 = -0.9222`)가 기록되어 있으나 `floor_detector.py`에는 `cv2.undistort()` 호출이 없습니다.
4. **`best.pt` vs 원본 데이터셋 부재**:
   - 학습 하이퍼파라미터와 100 에포크 곡선·최종 mAP는 완전히 복구되었으나, **`R3 #2`가 요구하는 총 이미지 장수, 클래스별 라벨 수, train/val/test 분할 비율, 출처 설명**은 Roboflow/학습 PC 또는 팀원 기록에서 별도 확인해야 합니다.

---

## 4. 리뷰어 3인 지적사항별 PC 자산 대응 가능성 매핑 (`R1~R3`)

| 분류 | 의미 | 해당 리뷰어 항목 | PC 보유 자산 / 필요 조치 |
| :---: | :--- | :--- | :--- |
| **(A) 즉시 대응 가능** | 추가 실행 없이 현재 추출 로그+수식/서술만으로 보강 | **`R3 #2` (부분)** | [`yolo11n_seg_train_results.csv`](../../experiments/logs/pc_extracted/yolo11n_seg_train_results.csv) + [`yolo11n_seg_checkpoint_meta.json`](../../experiments/logs/pc_extracted/yolo11n_seg_checkpoint_meta.json)으로 100ep 학습곡선/파라미터/Mask mAP50(`0.802`) 즉시 표기 (단, 데이터셋 장수·분할만 별도 확인) |
| **(A) 즉시 대응 가능** | 수식·텍스트 개정 | **`R2 #1, #2, #5, #9`**, **`R3 #5, Q1`**, **`R2 #4` (문헌)** | EMA($\alpha=0.6$)·Persistence($N=2 \Rightarrow 200\text{ ms}$) 지연 정식화, TCO 비교, `2.2–6.0 m` System Integration 포지셔닝 조정 |
| **(A) 선택 제외** | 주제 무관 인용 강요 | **`R1 #3`** | PatchTST·YOLO-TCS·손 열흔적 논문은 윤리 기준상 차기 투고 반영 제외 |
| **(B) 오프라인 실험 가능** | **추가 주행 없이** PC 기존 bag·카메라 프레임(`30,706장`) 재실행 | **`R1 #2`** | PC 내 `vlidar_run01..02` + Trash `vlidar_run01.2..01.6` 프레임에 캐시된 MiDaS/DA-V2 + 경량 MDE(`FastDepth`, `Lite-Mono` 등) 오프라인 실행 (yaw 보정 스크립트 수정 필수) |
| **(B) 오프라인 실험 가능** | 기존 `27,120장` Trash 카메라 bag 재실행 | **`R1 #4` (부분)**, **`R2 #3, #8, #10` (부분)** | **`nav_debug_avoidance`**(회전의자), **`vlidar_run01.3`**(카트+계단실 광원), **`vlidar_run01.4`**(창문 저각도 역광+박스), **`vlidar_run01.5`**(좁은 복도 정수기·점자블록·출입문 글레어, `floor_true=7.74%`) 프레임을 오프라인 재추론하여 **실제 환경 failure-mode 및 분할·스캔 안정성 사례** 도출 (단, GT가 물리 LiDAR가 아닌 당시 V-LiDAR `/scan`임은 명시 필요) |
| **(B) 오프라인 실험 가능** | 기존 bag 프레임 민감도·코어제한 벤치마크 | **`R2 #7`**, **`R2 #6` (부분)** | `c_conf`($0.15, 0.25, 0.40$) 변경 및 LUT 왜곡(`usb_webcam.yaml` 기반 `undistort` vs 미적용) 민감도 오프라인 분석; `taskset`/CPU 코어 제한 기반 저사양 CPU 추론 지연 시뮬레이션 |
| **(C) PC 로그만으로 불가** | 하드웨어 부재 또는 신규 실차 필수 | **`R3 #1`** | **물리 2D LiDAR 동시 비교** — 현재 `/dev/ttyLiDAR` 및 `ydlidar_ros2_driver`가 없어 PC 로그만으로는 불가능 (실제 YDLIDAR 연결·실차 필수) |
| **(C) PC 로그만으로 불가** | 신규 다중거리·유리문·보행자 실측 필수 | **`R3 #3`**, **`R1 #1`**, **`R1 #4` (유리/보행자)** | `2.2–6.0 m` 거리별 GT 계측, 카메라 $(h, \gamma)$ 실측 변화, **투명 유리벽·동적 보행자** 실차/스팟 로깅 필수 |
| **(C) PC 로그만으로 불가** | 수정 Nav2 설정 실차 재주행 필수 | **`R3 #4`** | `omo_r1.yaml` 최종 파라미터로 **동일 10 m 박스 코스 10회 재주행 Before/After 표** 작성 필수 (`Oct-2 minimal_run03..06`은 시퀀스/설정 검증 불완전) |

---

## 5. Git 푸시 가능 로그 구성 및 Trash 원본 백업 안내

### 5-1. `.gitignore` 정비 및 `experiments/logs/` 총량
- [`.gitignore`](../../.gitignore)에서 기존 `/experiments/logs/*` 일괄 차단을 **경량 텍스트/표 화이트리스트(`.csv`, `.md`, `.tex`, `.json`, `.yaml`, `.yml`, `.txt`)** 방식으로 변경하여 `.db3`, `.zstd`, `.pt`, `.mp4`, `.jpg` 등 대용량 바이너리는 계속 차단하면서 아래 로그를 모두 Git으로 푸시·동기화할 수 있습니다 (총 크기 **`1,133,543 bytes` / `1.2M`**):
  1. **기존 보유 로그 (`68 KB`)**:
     - `experiments/logs/baseline_benchmark_report.md`, `baseline_latex_table.tex`, `benchmark_run01.csv`, `benchmark_run02.csv`, `dynamic_trajectory_benchmark.csv`
     - `experiments/logs/20260903_171652_eval_test_run/eval_report.md`, `freespace_eval.csv`
  2. **신규 추출 PC 로그 ([`experiments/logs/pc_extracted/`](../../experiments/logs/pc_extracted/README.md), `~1.1 MB`)**:
     - 모델·캘리브레이션: `yolo11n_seg_train_results.csv`, `yolo11n_seg_checkpoint_meta.json`, `usb_webcam_calibration.yaml`, `openvino_model_metadata.yaml`
     - 전체/표 재현 통계: `rosbag_inventory_summary.csv` (40개 bag), `minimal_A_scen1_20runs_metrics.csv` (20회)
     - 10 Hz 정렬 궤적 CSV (`trajectories/*.csv` **14개**, 총 12,961 데이터 행 / 헤더 포함 12,975줄):
       `active_minimal_A_scen2_run01.csv`, `active_minimal_run03..06.csv`, `active_vlidar_run01..02.csv`, `trash_nav_debug_avoidance.csv`, `trash_vlidar_run01.csv`, `trash_vlidar_run01.2..01.6.csv`

### 5-2. [선택] Trash 원본 카메라 Rosbag 안전 백업 명령 (휴지통 비우기 방지)
`pc_extracted/trajectories/trash_*.csv`로 스칼라 궤적은 보존되었으나, 추후 오프라인 MDE/세그멘테이션 재추론(`R1 #2`, `R2 #7`, failure-mode 시각화)을 위해 **원본 이미지가 포함된 Trash bag 6종(`nav_debug_avoidance`, `vlidar_run01.2~01.6`)**을 계속 사용하려면 OS 휴지통 비우기 전에 별도 백업 경로로 옮겨 두는 것을 권장합니다 (`~/data/`는 `.gitignore`로 이미 무시됨):

```bash
mkdir -p ~/data/bags_trash_archive
mv ~/.local/share/Trash/files/nav_debug_avoidance \
   ~/.local/share/Trash/files/vlidar_run01 \
   ~/.local/share/Trash/files/vlidar_run01.2 \
   ~/.local/share/Trash/files/vlidar_run01.3 \
   ~/.local/share/Trash/files/vlidar_run01.4 \
   ~/.local/share/Trash/files/vlidar_run01.5 \
   ~/.local/share/Trash/files/vlidar_run01.6 \
   ~/data/bags_trash_archive/
```
