# [교수님 보고용] V-LiDAR 후속 투고 전략 · 코드 패치 검증 · 축소 실차 계획 요약

- **작성/갱신 일자**: `2026-11-24` (원본 `2026-11-13` 요약본에 로봇 PC 코드 패치·실측 재평가·저널 정책 교차검증 반영)
- **이전 심사 배경**:
  - **IEEE Access (`Access-2026-51740`, `2026-11-12` Reject)**: Major Revision 없는 저널 구조 + 물리 2D LiDAR 미비교, YOLO 학습 프로토콜 누락, 단일 홀/단일 거리(`2.50 m`), `run08/12` 수정 후 재주행 미제시 지적.
  - **IJCAS AE 코멘트 (`docs/reviews/comment.txt`)**: 경량성 및 한계 명시(`<= 0.261 m`, `78% = 39/50`) 호평, 단 ① 단일 복도·단일 박스 예비 수준, ② `22%` 실패율, ③ 광택 바닥/조명 민감도, ④ **`"comparison against a LiDAR or alternative baseline is missing"`** 지적.
- **연계 문서**: [06_Advisor_Agreed_Scope_and_Next_Steps.md](./06_Advisor_Agreed_Scope_and_Next_Steps.md) · [09_IJCAS_Review_Priority_and_3D_LiDAR_Option_20261118.md](./09_IJCAS_Review_Priority_and_3D_LiDAR_Option_20261118.md) · [05_Reduced_40Run_Pilot_and_Calibration_Checklist.md](./planning/05_Reduced_40Run_Pilot_and_Calibration_Checklist.md)

---

## 1. 차기 타겟 저널 선정안 (**SCIE Q2 이상 · 비-MDPI · 300만 원 이내 · 물리 2D LiDAR 미사용**)

현재 로봇 PC(`/home/cgv`)에 **물리 2D LiDAR 실물·드라이버(`/dev/ttyLiDAR`, `ydlidar_ros2_driver`)가 부재**하며, 교수님 지침(**게재료 300만 원 이내 · 빠른 정리**)과 **2025 JCR Q2 이상(비-MDPI)** 조건에 맞춰 교차 검증한 결과입니다.

### 1-1. 제출 가능 SCIE Q2+ 저널 비교표 (1군 우선 · 2군 대안)

| 군 | 저널명 (출판사 · ISSN) | 등급 (WoS / Scopus) | 비용 / 분량 제한 | 1차 판정(추적) | 심사 방식 / 저장소 초안 현황 | 포지셔닝 및 적합성 |
| :-: | :--- | :---: | :---: | :---: | :---: | :--- |
| **1군** | **IJCAS**<br>*(Springer/ICROS·KIEE, 1598-6446)* | WoS **Q2** (`JIF ~2.8`) | **`$300`** *(2026~ 페이지차지 폐지)* | **중앙값 ~55일** (`~8주`) | **Single-blind**<br>✅ [IJCAS_Manuscript.pdf](../../IJCAS/ICCAS/IJCAS_Manuscript.pdf) (`9쪽`, `492행`) | **최우선 추천**: 기존 AE 코멘트(`LiDAR or alternative baseline`)와 가장 직접 연결되며 `exp/20261123-ijcas-p0` 브랜치 정비 완료 *(단, 구버전 `78%`/`0.261 m` 본문을 새 실측으로 전면 갱신 필요)* |
| **1군** | **JRTIP**<br>*(Springer, 1861-8200)* | WoS **Q2** / Scopus Q1~Q2 (`IF ~4.9`) | **`$0`** (Hybrid 구독) · 일반 논문 별도 고정 쪽수 제한 없음 | **평균 ~5주** (`~35일`) | **Double-blind (익명화 필수)**<br>❌ 템플릿 미보유 | **무료+초고속 대안**: Nav2보다 **OpenVINO `70.5 FPS` 실시간성 + 역광 오탐 `32%` 차단** 등 영상처리 기여 중심으로 재구성 시 적합 |
| **1군** | **ISR**<br>*(Springer/KROS, 1861-2776)* | WoS **Q2** / Scopus Q1 (`IF ~4.3–5.9`) | **`$0`** (Hybrid 구독) · 일반 논문 고정 쪽수 제한 없음 | **평균 2~4+개월** | **Double-blind (익명화 필수)**<br>❌ 템플릿 미보유 | **주제 적합도 최상**: OMO-R1 서비스 로봇·Nav2와 가장 잘 맞고 KROS 연계이나, **익명화 공수와 긴 1차 심사**가 단점 |
| **2군** | **IOP MST**<br>*(IOP, 0957-0233)* | WoS **Q1/Q2 경계** / Scopus Q2 (`JIF ~3.7`) | **`$0`** (Hybrid 구독) · Note ~2,500단어/3p 외 정규 제한 없음 | 초기 triage 빠름 *(약 5~9주)* | **Single/Double 선택 가능**<br>✅ [IOP_MST/main.pdf](../../paper/workspace/IOP_MST/main.pdf) (`11쪽`, `299행`) | **무료 계측 대안**: 단안 LUT 오차(`0.152 m`) + D1/D2 광학 불확실성 강조 *(단, 기존 초안은 구버전 `78.4 FPS Arc GPU`/`0.261 m` 수치라 교체 필수)* |
| **2군** | **Mechatronics**<br>*(Elsevier, 0957-4158)* | WoS **Q2** (`IF ~3.2–3.8`) | **`$0`** (Hybrid 구독) | 약 `6~10주` | 통상 Single-blind<br>❌ 템플릿 미보유 | ROS 2 + 사각지대 보호형 Nav2(`raytrace_min_range: 2.18`) 메카트로닉스 통합 포지셔닝 |
| **2군** | **Elsevier C&EE**<br>*(Elsevier, 0045-7906)* | WoS **Q1** / Scopus Q1 (`IF ~4.9–5.5+`) | **`$0`** (Hybrid 구독) | 약 `4~8주` *(전체 `2~4+개월`)* | 통상 Single-blind<br>✅ [Elsevier_CEE/main.pdf](../../paper/workspace/Elsevier_CEE/main.pdf) (`24쪽` preprint, `475행`) | Q1 무료 대안이나 요구 수준 높음 *(기존 초안 역시 구버전 `0.261 m`/`18/20` 수치라 전면 갱신 필수)* |
| **2군** | **AURO**<br>*(Springer, 0929-5593)* | WoS **Q2** (`IF ~4.3`) | **`$0`** (Hybrid 구독) | **약 4~5개월 (`~19주`)** | ❌ 템플릿 미보유 | 정통 로봇 저널이나 심사 기간이 길고 기준이 높아 빠른 마무리에는 비추천 |

### 1-2. 제외 확정 저널 및 사유 (보고 참고용)
* **PeerJ Computer Science (`2376-5992`)**: JCR Q2·1차 ~5주이나 Fully OA APC **`$2,370`**(+세금, 약 330만 원+)으로 300만 원 예산 초과.
* **IEEE Sensors Journal (`1530-437X`)**: Hybrid OA **`$2,800`**(약 390만 원) 예산 초과, 구독 모델도 **8쪽 초과 시 쪽당 `$175` 필수 부과**, 1차 6~12주 및 물리 LiDAR 부재 리스크.
* **IEEE Sensors Letters (`2475-1472`)**: **ESCI** (SCIE 미등재) 및 4쪽 제한.
* **ETRI Journal (`1225-6463`) / Robotica (`0263-5747`)**: 무료 구독/OA(`$0`)이나 WoS JCR 기준 **Q3** (`IF ~2.0` / `~2.9`, 단 Scopus상으로는 Q2)로 WoS Q2 기준 미달.
* **MDPI 계열 (`Sensors` 등) / IEEE Access**: MDPI 제외 방침(+ `2,600 CHF`) / IEEE Access 재제출 금지 Reject.
* *(참고 교정)* **JIRS (`0921-0296`)**: 1차 중앙값 **12일**로 매우 빠르나 완전 유료 OA 전용이 아니라 **Springer Hybrid(`$0` 구독 / `~$3,190` OA)**로 확인됨. 단, Clarivate `2025 JCR` Q2 충족 여부는 도서관 원문 확인 필요.

> **확인 필요 사항**: 1순위 **IJCAS** 확정 시 포털 판정 메일 원문에서 이번 제출이 **재투고(Resubmit)** 트랙인지 **신규 투고(New Submission)**인지 메일함에서 최종 확인이 필요합니다.

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
