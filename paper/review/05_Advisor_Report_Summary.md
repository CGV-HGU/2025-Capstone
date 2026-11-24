# [교수님 보고용] V-LiDAR 후속 투고 전략 · 코드 패치 검증 · 축소 실차 계획 요약

- **작성/갱신 일자**: `2026-11-24` (원본 `2026-11-13` 요약본에 로봇 PC 코드 패치·실측 재평가·저널 정책 교차검증 반영)
- **이전 심사 배경**:
  - **IEEE Access (`Access-2026-51740`, `2026-11-12` Reject)**: Major Revision 없는 저널 구조 + 물리 2D LiDAR 미비교, YOLO 학습 프로토콜 누락, 단일 홀/단일 거리(`2.50 m`), `run08/12` 수정 후 재주행 미제시 지적.
  - **IJCAS AE 코멘트 (`docs/reviews/comment.txt`)**: 경량성 및 한계 명시(`<= 0.261 m`, `78% = 39/50`) 호평, 단 ① 단일 복도·단일 박스 예비 수준, ② `22%` 실패율, ③ 광택 바닥/조명 민감도, ④ **`"comparison against a LiDAR or alternative baseline is missing"`** 지적.
- **연계 문서**: [06_Advisor_Agreed_Scope_and_Next_Steps.md](./06_Advisor_Agreed_Scope_and_Next_Steps.md) · [09_IJCAS_Review_Priority_and_3D_LiDAR_Option_20261118.md](./09_IJCAS_Review_Priority_and_3D_LiDAR_Option_20261118.md) · [05_Reduced_40Run_Pilot_and_Calibration_Checklist.md](./planning/05_Reduced_40Run_Pilot_and_Calibration_Checklist.md)

---

## 1. 차기 타겟 저널 확정안: **IJCAS 집중** (물리 2D LiDAR 미사용)

현재 로봇 PC(`/home/cgv`)에 **물리 2D LiDAR 실물·드라이버(`/dev/ttyLiDAR`, `ydlidar_ros2_driver`)가 부재**하고, 교수님 지침(**게재료 300만 원 이내 · 빠른 정리**)을 반영한 비교 결과는 다음과 같습니다.

| 후보 저널 | 비용 (APC) | 심사 특징 / 속도 | 블라인드 | 적합성 판단 |
| :--- | :---: | :--- | :---: | :--- |
| **IJCAS (1순위 추천)** | **`$300` (약 42만 원)** *(2026~ 페이지차지 폐지)* | 1차 약 `1.5~3개월` / 전체 `4~6개월` | Single-blind | **가장 현실적**: AE 코멘트(`LiDAR or alternative baseline`)에 맞춰 **MDE 대조 + 다환경 실차**로 방어 가능하며, `exp/20261123-ijcas-p0` 원고·실험 계획이 이미 준비됨 |
| **ETRI Journal (대안)** | **`$0` (무료)** | 1차 메타 `~8주` / 실제 승인 `3~6개월` | Double-anonymous | 비용은 유리하나 **이중 익명화(학교·로봇 로고 제거)** 공수 및 ICT 중심 스코프 적합성 점검 필요 |
| **IEEE Sensors Journal (비추천)** | **OA `$2,800`** *(구독 8p 초과 시 `$175`/p)* | 1차 약 `6~12주` | Single-blind | Q1 센서 저널이라 물리 LiDAR 없이 리젝 위험이 높고, OA 기준 **300만 원 예산 초과** 및 8쪽 압축 부담 |

> **확인 필요 사항**: IJCAS 포털 판정 메일 원문에서 이번 재제출이 **재투고(Resubmit)** 트랙인지 **신규 투고(New Submission)**인지 메일함에서 최종 확인이 필요합니다.

---

## 2. 코드 결함 패치 및 완전 재현 벤치마크 결과 (`2026-11-24` 완료)

기존 IEEE Access 원고의 미연결 수치(`0.192 m`, `12.9 ms`, `18/20` 미검증 주행)를 **전면 폐기**하고, 로봇 PC 실제 코드([floor_detector.py](../../src/freespace_detection/freespace_detection/floor_detector.py), [fake_lidar_with_tf.py](../../src/fake_lidar_with_tf/fake_lidar_with_tf/fake_lidar_with_tf.py), [omo_r1.yaml](../../src/omo_r1_navigation2/param/omo_r1.yaml), [benchmark_depth_baselines.py](../../experiments/benchmark_depth_baselines.py))의 치명적 결함 4종을 수정한 뒤 재측정했습니다.

### 2-1. 핵심 코드 패치 및 안전성 검증 완료
1. **OpenVINO FP16 CPU 실제 연결 + `cls=0` 바닥 라벨 역전 수정 & 보행자 분리 검증 PASS**:
   - `best.pt` / `best_openvino_model` 메타데이터는 `{0: 'person', 1: 'floor'}`이지만 실제 학습 가중치는 **바닥을 `cls=0`으로 예측**함을 실측으로 발견(`cls=1`은 80프레임 중 0건).
   - `floor_class_id=0` 명시 매핑 및 상단(`y < 120`) 고립 오탐 차단 가드(`min_floor_scan_pixels=50`)를 적용했고, 실제 성인 보행자 포함 이미지([experiment1_setup.jpeg](../../IJCAS/ICCAS/IJCAS/paper/experiment1_setup.jpeg), [live_view.png](../../IJCAS/ICCAS/IJCAS/paper/live_view.png))에서 **보행자 전신·박스·쓰레기통이 비바닥(장애물)으로 정상 분리**됨을 확인했습니다.
2. **사각지대(`2.18 m`) 장애물 보존형 Costmap 관리 & Stale Scan 차단**:
   - 하단 ROI만 보고 15초마다 costmap 전체를 삭제하던 `clear_all_costmaps()` 자동 호출을 기본 비활성화(`enable_auto_costmap_clear=False`)했습니다.
   - `omo_r1.yaml` (`local_costmap`·`global_costmap`)에 `raytrace_min_range: 2.18`(사각지대 셀 보존), `inf_is_valid: False`, `expected_update_rate: 0.5`(10 Hz 기준 0.5초 타임아웃), `observation_persistence: 0.0`을 적용했습니다.
   - `floor_detector.py`가 카메라 획득 `header.stamp`를 보존해 `/scan`을 직접 발행하도록 변경하고, `fake_lidar_with_tf.py`는 `lidar_x_offset` TF 파라미터화 및 `0.5 s` stale 만료 차단 릴레이로 정비했습니다.
3. **벤치마크 평가 버그 및 MDE 상대 깊이 보정 수정**:
   - `benchmark_depth_baselines.py`의 `GT + 0.35` 및 예외 고정상수 fallback을 제거(미검출 시 `NaN`/`not_measured`)하고, `/odom` yaw 추적·방위각 창 정렬·센서 외인자 보정(`--sensor-offset-x`)을 반영했습니다.
   - `Depth Anything V2 Small`의 기존 `3.277 m` 오차가 `predicted_depth`(상대 역깊이·disparity)에 선형 `min-max` 정규화를 걸어 **원근 부호가 정반대로 뒤집힌 버그** 때문임을 확인·수정했습니다. 공정성을 위해 `MiDaS`와 `DA-V2 Small` 모두 **2-fold 교차 어파인 보정(`1/d = s·disp + t`, `run01` 보정→`run02` 평가 및 역방향)**을 적용해 Out-of-Sample MAE를 산출했습니다.
   - *(정정)* 기존 `run08/12` 실패 사례는 *"비전 100% 정상 확정"*이 아니라 ***"하단 ROI 95% 채움 조건(`/floor_detector=True`)은 유지되었으나 영상·경로·costmap 토픽 미기록으로 비전 전체 정상 단정 불가, Costmap/파라미터(`raytrace_min_range: 2.18`, `clear_all_costmaps` 비활성화) 보완 필요"***로 서술을 정정했습니다.

### 2-2. 재현 가능한 동적 벤치마크 비교표 (`vlidar_run01` + `vlidar_run02` 총 200프레임)
원본 로그: [`baseline_benchmark_report.md`](../../experiments/logs/baseline_benchmark_report.md) · [`baseline_latex_table.tex`](../../experiments/logs/baseline_latex_table.tex) (모델별 **독립 추론 루프**로 CPU 캐시 간섭 제거, 프레임 0 콜드 웜업 제외 steady-state 기준, `n=59/59` 유효 구간 `2.18–5.0 m`, `in_fov=1`)

| 모델 / 기법 | 백엔드 (CPU) | Params | Steady 지연 (`mean ± std`) | P50 / P95 (ms) | Steady FPS | 동적 MAE (m) | 유효율 |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **V-LiDAR (제안)** | OpenVINO FP16 | `2.84 M` | **`14.18 ± 1.97 ms`** | `14.1 / 17.2` | **`70.5 Hz`** | **`0.152 m`** | `100.0%` |
| **MiDaS v2.1 Small** | PyTorch CPU | `21.32 M` | `36.53 ± 3.12 ms` | `35.4 / 42.8` | `27.4 Hz` | `0.811 m` *(2-fold OOS)* | `100.0%` |
| **Depth Anything V2 Small** | PyTorch CPU | `24.79 M` | `570.06 ± 12.85 ms` | `567.4 / 596.7` | `1.8 Hz` | `0.519 m` *(2-fold OOS)* | `100.0%` |

*(참고: `Lite-Mono`, `FastDepth`는 현재 로봇 PC에 패키지·체크포인트 캐시가 없어 이번 오프라인 실행에서 제외됨. `V-LiDAR`는 `DA-V2 Small` 대비 정확도 `3.4×`(`0.152 m` vs `0.519 m`), 추론 속도 `40.2×`(`14.18 ms` vs `570.06 ms`) 우위를 공정한 동일 기준에서 입증.)*

### 2-3. 글레어 · 역광 · 좁은 복도 실패 분석 요약 ([`glare_backlight_failure_audit.csv`](../../experiments/logs/glare_backlight_failure_audit.csv))
| Bag (장면) | 샘플 수 | 단독 OpenVINO Steady 지연 | 평균 바닥 면적 | ROI 통과율(`>=95%`) | 상단 오탐 차단(`y<120`) | 핵심 진단 |
| :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| `vlidar_run01` (기준 홀+박스) | 100 / 2,034 | `13.34 ms` (P95 `16.51`) | `30.15%` | `88.0%` | `0` | 안정적 기준 조건 |
| `vlidar_run02` (기준 홀+박스) | 100 / 1,552 | `14.05 ms` (P95 `17.03`) | `28.23%` | `88.0%` | `1` | 근접/가림 구간 `zero_floor` 12프레임 포함 |
| `nav_debug_avoidance` (회전의자) | 100 / 1,971 | `14.11 ms` (P95 `16.13`) | `29.33%` | `73.0%` | `0` | 의자 다리·근접 회피로 정면 `<2.2 m` 빈도 `25%` |
| `vlidar_run01.3` (금속 카트+계단광) | 100 / 2,613 | `13.94 ms` (P95 `16.86`) | `29.72%` | `62.0%` | `2` | 계단실 눈부심·카트 구조로 ROI 통과율 하락 |
| `vlidar_run01.4` (창문 저각도 역광+박스) | 100 / 2,305 | `13.81 ms` (P95 `16.96`) | `35.31%` | `86.0%` | **`32`** | **상단(`y<120`) 반사/역광 고립 오탐 32%를 패치 가드로 전부 차단** |
| `vlidar_run01.5` (좁은 복도+출입구 글레어) | 200 / 15,895 | `14.34 ms` (P95 `17.76`) | **`17.22%`** | **`7.0%`** | `0` | 좁은 복도·점자블록·글레어 환경에서 운용 한계 정량 확인 |

---

## 3. 실속형 축소 실차 실험 및 파일럿 계획

기존 `proposal_v2_20261118`의 3장소(60회) 계획을 일정에 맞춰 **명확히 대비되는 2장소 × 2배치 × 10회 = 총 40회** + **D1/D2 정적 클립(줄자/거리계 독립 GT)**으로 압축했습니다.

1. **줄자 실측 및 파일럿 1회 선행**: [`05_Reduced_40Run_Pilot_and_Calibration_Checklist.md`](./planning/05_Reduced_40Run_Pilot_and_Calibration_Checklist.md) 절차에 따라 `base_link` ↔ 카메라 렌즈 X 오프셋 실측(`lidar_x_offset` 고정) 후, 개정된 [`scripts/record_rosbag.sh`](../../scripts/record_rosbag.sh)(ZSTD 압축 영상 + `/scan`, `/odom`, `/plan`, `local/global costmap`, `/navigate_to_pose/_action/status`)와 [`scripts/test_avoidance_goal.py`](../../scripts/test_avoidance_goal.py)(`--out-json` 결과 저장)로 누락 없는 1회 파일럿 검증.
2. **실차 본 평가 (40회)**:
   - **장소 A (균일 조명 · 저반사 직선)** vs **장소 B (광택 바닥 · 창가 반사/역광)**
   - **S1 (단일 박스 `(4.5, 0)m`)** vs **S2 (복수·이형 장애물: 박스 `(3.5, +0.35)m` + 의자/카트 `(7.0, -0.35)m`)**, 각 조건 10회(실패·타임아웃·개입 전수 포함).
3. **정적 비교 (D1/D2)**: 줄자/레이저 거리계 독립 GT로 **MDE 보정 split(배치 1)과 평가 split(배치 2·3)을 분리**하고 조명 조건별 오탐·미검출 분포 확보.
