# Robot PC Extracted Experiment Logs (`pc_extracted`)

로봇 구동 PC(`/home/cgv`)의 바이너리 자산(`.db3`, `.db3.zstd`, `best.pt`, 캘리브레이션 YAML)에서 Git 푸시 및 오프라인 재분석이 가능하도록 경량 메타데이터와 스칼라 시계열만 추출해 정리한 디렉터리입니다. (`/camera/image_raw`, `.db3`, `.mp4`, `.pt` 등 대용량 바이너리는 제외됨.)

---

## 1. 파일 목록

| 파일 / 폴더 | 추출 원본 (로봇 PC) | 내용 설명 |
| :--- | :--- | :--- |
| **`yolo11n_seg_train_results.csv`** | `src/freespace_detection/scripts/best.pt` (`train_results`) | YOLO11n-seg 100 에포크 전체 학습 곡선 (box/seg/cls/dfl loss, Box/Mask P/R/mAP50/mAP50-95, lr; 101행) |
| **`yolo11n_seg_checkpoint_meta.json`** | `src/freespace_detection/scripts/best.pt` (`train_args`, `train_metrics`) | 체크포인트 MD5 (`4e8a79093d9ca3befa76fde5e4f84115`), 학습 날짜(`2024-10-24`), Ultralytics `8.3.4`, 클래스(`person`, `floor`), 최종 지표 및 하이퍼파라미터 |
| **`usb_webcam_calibration.yaml`** | `/home/cgv/data/cam/usb_webcam.yaml` | USB 웹캠 내부 파라미터 (`640x480`, `fx=517.20`, `fy=513.18`, `cx=318.92`, `cy=252.41`, `k1/k2/p1/p2/k3`) |
| **`openvino_model_metadata.yaml`** | `src/freespace_detection/scripts/best_openvino_model/metadata.yaml` | OpenVINO FP16 변환 메타데이터 (`2026-09-30`, Ultralytics `8.4.47`, `imgsz=[256, 320]`, `half=true`) |
| **`rosbag_inventory_summary.csv`** | `/home/cgv/data/bags/*` (28개) + `~/.local/share/Trash/files/*` (12개) | 총 40개 rosbag의 메타데이터, 토픽 수, 주행 시간, `/odom` 이동 통계, `/scan` 최소 거리, `/floor_detector` 비율 종합표 |
| **`minimal_A_scen1_20runs_metrics.csv`** | `/home/cgv/data/bags/minimal_A_scen1_run01..20` | 논문 Table III 보고 20회 실차 주행(`run01..20`)의 개별 텔레메트리 재현 지표 (`run08`, `run12` 정지 사례 포함) |
| **`trajectories/*.csv`** (14개) | 미보고 `/home/cgv/data/bags/` 및 `Trash/files/` 주요 bag | `/scan` 타임스탬프(~10 Hz) 기준 `/odom`, `/cmd_vel`, `/floor_detector`, 좌/중앙/우 섹터 최소거리 정렬 시계열 |

---

## 2. `rosbag_inventory_summary.csv` 주요 컬럼 정의

- **`source_group`**: `active_data_bags` (`/home/cgv/data/bags/`) vs `trash_files` (`/home/cgv/.local/share/Trash/files/`)
- **`status`**:
  - `OK_DB3`: 비압축 `.db3` 직접 읽기 성공
  - `OK_ZSTD`: `.db3.zstd` 임시 해제 후 읽기 성공 (원본 무변경, 임시 파일 자동 삭제)
  - `CORRUPT_ZSTD: ...`: zstd 스트림 손상 (`minimal_run02_0.db3.zstd` premature end)
- **`duration_s`**: `/tf_static` 제외 첫 메시지~마지막 메시지 전체 시간 (`s`)
- **`odom_moving_dt_s`**: `/odom` 연속 샘플 중 `|vx| > 0.01 m/s` 또는 `|wz| > 0.02 rad/s`인 구간 시간 합 (`s`)
- **`cmd_active_dt_s`**: `/cmd_vel` 명령이 영(0)이 아닌 활성 제어 시간 합 (`s`, `/cmd_vel` 미기록 bag은 공란)
- **`net_disp_m` / `path_length_m` / `max_abs_y_m`**: `/odom` 시작-끝 직선 변위, 누적 경로 길이, 최대 횡방향 이탈
- **`global_min_scan_m`**: `/scan` 전체 채널(`range_min <= r <= range_max`, 유한값) 중 최소 감지 거리 (`m`)
- **`floor_true_pct`**: `/floor_detector` 메시지 중 `True` 비율 (`%`)

---

## 3. `trajectories/*.csv` 컬럼 정의 (15개 컬럼)

모든 행은 각 bag의 `/scan` 메시지(~10 Hz) 타임스탬프를 기준으로 동기화되었습니다.

| 컬럼 | 설명 |
| :--- | :--- |
| `t_s` | bag 시작 시각 대비 경과 시간 (`s`) |
| `x_m`, `y_m`, `yaw_deg` | 가장 가까운 `/odom` 위치 (`m`) 및 요 각도 (`deg`) |
| `vx_mps`, `wz_rps` | `/odom` 선속도 (`m/s`) 및 각속도 (`rad/s`) |
| `cmd_vx_mps`, `cmd_wz_rps` | 직전 `/cmd_vel` 지령 (1초 이내 유효값, 토픽 미포함 시 공란) |
| `floor_ok` | 최근접 `/floor_detector` (`1`=True, `0`=False, 0.5초 초과 시 공란) |
| `finite_channels` | 141개 채널 중 유효(`range_min <= r <= range_max`, finite) 채널 수 |
| `min_scan_m` | 전방위 유효 채널 최소 거리 (`m`) |
| `right_min_m` | 우측 섹터(`angle <= -12 deg`) 최소 거리 (`m`) |
| `center_min_m` | 중앙 섹터(`-12 < angle < +12 deg`) 최소 거리 (`m`) |
| `left_min_m` | 좌측 섹터(`angle >= +12 deg`) 최소 거리 (`m`) |
| `min_scan_angle_deg` | `min_scan_m`이 측정된 채널의 각도 (`deg`, `-35`~`+35`) |

---

## 4. 주의사항

1. **Trash 원본 보관 경고**: `trash_*` 파일들의 원본 `.db3`/`.db3.zstd`는 현재 `/home/cgv/.local/share/Trash/files/`에 있으므로 OS 휴지통 비우기 시 원본 영상/카메라 프레임이 영구 삭제될 수 있습니다. 필요한 원본은 별도 백업 디렉터리로 옮겨 보관하시기 바랍니다.
2. **기존 `experiments/logs/` 파일과의 관계**: 상위 디렉터리 [`../`](../)에는 기존 논문/벤치마크 작성 시 생성된 `benchmark_run01.csv`, `benchmark_run02.csv`, `dynamic_trajectory_benchmark.csv`, `baseline_benchmark_report.md`, `baseline_latex_table.tex`, `20260903_171652_eval_test_run/`이 함께 추적 가능하도록 `.gitignore`가 정비되어 있습니다.
