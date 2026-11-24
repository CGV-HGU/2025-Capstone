# 실속형 40회 실차 실험 축소안 및 파일럿·줄자 실측 체크리스트

- 작성일: 2026-11-24
- 기준 코드: 패치 완료 버전 (`floor_detector.py` OpenVINO + `cls=0` + 직발행 `/scan`, `fake_lidar_with_tf.py` TF/stale 차단, `omo_r1.yaml` `raytrace_min_range: 2.18` / `expected_update_rate: 0.5`)
- 상위 계획: [04_Robot_Checked_Code_and_Experiment_Plan_20261118.md](./04_Robot_Checked_Code_and_Experiment_Plan_20261118.md) (기존 60회/3장소 풀코스의 실차 일정 압축 실행안)

---

## 1. 실차 축소 실험 구성 (총 40회 주행 + D1/D2 정적 클립)

| 구분 | 구성 | 목적 / 대응 리뷰 |
| :--- | :--- | :--- |
| **폐루프 주행 (40회)** | **2장소 × 2배치(S1/S2) × 각 10회** | 단일 복도·단일 박스 한계 + 실패율 + Nav2 개선 검증 |
| ├ **장소 A (기준)** | 균일 조명 · 저반사 실내 직선 구간 | 기본 조건에서의 단일(S1) / 복수·이형(S2) 회피 성능 |
| ├ **장소 B (악조건)** | 광택 바닥 · 창가 반사광/역광 구간 | IJCAS AE 지적 광택 바닥/조명 민감도 실차 검증 |
| ├ **배치 S1** | 단일 박스 중심 `(4.5, 0)m` → 목표 `(10.0, 0)m` | 기존 `run01..20` 대비 개선된 설정(`raytrace_min_range: 2.18`) 검증 |
| └ **배치 S2** | 복수·이형 장애물: 박스 `(3.5, +0.35)m` + 의자/카트 `(7.0, -0.35)m` | 박스 외 형상 및 연속 장애물 재계획 검증 |
| **정적 비교 (D1/D2)** | **D1** `3거리(2.5/3.5/4.5m) × 3방위(-20/0/+20°) × 3배치` (또는 시간 부족 시 `0°` 정면 `3거리 × 3배치 = 9클립` 우선) + **D2** `3배치 × 3조명 = 9클립` | 줄자/거리계 독립 GT 기반 MDE 보정(배치1) · 평가(배치2·3) 분리 및 조명 영향 정량화 |

> **참고**: `proposal_v2_20261118`의 60회(3장소) 중 3번째 장소(장소 C)를 생략하는 대신, 장소 A와 B의 **조명·바닥 재질 대비**가 명확하도록 선정하고 모든 시도의 실패·타임아웃·수동 개입을 누락 없이 기록합니다.

---

## 2. 실차 전 줄자 실측 체크리스트 (`lidar_x_offset` 및 GT 원점 통일)

과거 `fake_lidar_with_tf.py`는 `lidar_x_offset = -0.55 m`, `omo_r1.urdf`의 `camera_joint`는 `x = -0.50 m`, `benchmark_run01/02`의 `obs_x`는 카메라/전면 기준 등으로 기준점이 혼재했습니다. 실차 시작 전 아래 4항목을 줄자로 1회 실측해 값을 고정합니다.

- [ ] **1. `base_link` 원점 확인**: OMO-R1 양쪽 구동 바퀴 축 중심(`base_link` 원점) 위치를 차체 측면에 마스킹 테이프로 표시
- [ ] **2. 카메라 렌즈 X 오프셋 실측**: `base_link` 기준 전방(+) / 후방(-) 방향으로 실제 웹캠 렌즈 앞면까지의 수평 거리 $\Delta x_{\text{cam}}$ 측정 (예: `-0.50 m` 또는 `-0.55 m` 또는 `+0.xx m`)
- [ ] **3. 카메라 높이($h$) 및 틸트각($\gamma$) 재확인**: 지면→렌즈 중심 높이(기존 `1.05 m`), 피치 하향 각도(기존 `2.0°`) 실측 후 `generate_luts.py` 설정과 일치하는지 확인
- [ ] **4. 파라미터 즉시 반영**: 측정한 $\Delta x_{\text{cam}}$을 실행 명령에 그대로 전달 (별도 코드 수정 불필요)
  ```bash
  ros2 launch robot_launch_package multi_node_launch.py lidar_x_offset:=-0.50
  ```
- [ ] **5. D1·E1 기준점 통일**: 정적 GT 거리(`2.5 / 3.5 / 4.5 m`) 및 주행 시작선 `(0, 0)`은 **카메라 렌즈 위치 기준인지 `base_link` 기준인지** 기록표에 명시하고, `benchmark_depth_baselines.py --sensor-offset-x` 값과 일치시킴

---

## 3. 파일럿 1회 주행 및 Rosbag 누락 점검 절차

과거 `minimal_A_scen1_run01..20`처럼 영상·경로·costmap·action 결과가 빠지는 사고를 막기 위해, 수정된 [`scripts/record_rosbag.sh`](../../../scripts/record_rosbag.sh)와 [`scripts/test_avoidance_goal.py`](../../../scripts/test_avoidance_goal.py)를 함께 사용합니다.

### 실행 순서 (파일럿 1회)

1. **로봇 스택 실행** (터미널 1):
   ```bash
   cd ~/ros2_ws && source install/setup.bash
   ros2 launch robot_launch_package multi_node_launch.py lidar_x_offset:=<실측값> enable_gui:=false
   ```
2. **전체 토픽 ZSTD 녹화 시작** (터미널 2):
   ```bash
   cd ~/ros2_ws
   ./scripts/record_rosbag.sh nav pilot_placeA_S1_01
   ```
   - 포함 토픽: `/camera/image_raw`, `/camera/camera_info`, `/scan`, `/lidar_channel_distances`, `/floor_detector`, `/tf`, `/tf_static`, `/odom`, `/cmd_vel`, `/plan`, `/local_plan`, `/local_costmap/costmap`, `/local_costmap/costmap_updates`, `/global_costmap/costmap`, `/initialpose`, `/goal_pose`, `/navigate_to_pose/_action/status`, `/navigate_to_pose/_action/feedback`, `/rosout`
3. **Nav2 목표 전송 및 결과 JSON 자동 저장** (터미널 3):
   ```bash
   cd ~/ros2_ws && source install/setup.bash
   python3 scripts/test_avoidance_goal.py -x 10.0 -t 120.0 \
     --trial-id pilot_placeA_S1_01 \
     --out-json ~/data/bags/nav_pilot_placeA_S1_01/trial_outcome.json
   ```
4. **녹화 종료(Ctrl+C) 직후 자동 검증 항목**:
   - `record_rosbag.sh` 종료 시 자동 출력되는 `ros2 bag info`에서 **`/camera/image_raw`, `/scan`, `/odom`, `/plan`, `/local_costmap/costmap`, `/navigate_to_pose/_action/status` 메시지 수 > 0** 확인
   - `trial_outcome.json` 내 `nav_status` (`SUCCEEDED` / `TIMEOUT` / `STATUS_*`), `pure_motion_s`, `travel_x_m` 기록 확인
   - `df -h ~/data/bags` 잔여 디스크 용량 확인 (현재 ZSTD 압축 적용; 40회 본 평가 전 여유 용량 확보)
