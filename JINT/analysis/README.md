# JINT 투고 준비 및 자료 대조

2026-11-26 KST · Journal of Intelligent & Robotic Systems · 내부 검토용

- [투고 전략 및 제출 판단 기준](../planning/2026-11-27_JINT_submission_strategy.md) · [초안 체크리스트](../planning/JINT_draft_checklist.md) · [영문 LaTeX 원고](../manuscript/main.tex) · [검토용 PDF](../JINT_first_draft.pdf) · [검토용 소스 ZIP](../manuscript/JINT_draft_source.zip) · [커버레터 초안](../planning/cover_letter_draft.md)
- [Springer Nature 공식 양식 ZIP](../template/springer-nature-latex-2024-12.zip) · [양식 안내](https://www.springernature.com/la/authors/campaigns/latex-author-support) · [JINT 투고 지침](https://link.springer.com/journal/10846/submission-guidelines)
- [PC rosbag 목록](evidence/rosbag_inventory.csv) · [과거 비교 bag의 RGB 표본](evidence/benchmark_rgb_contact_sheet.jpg) · [기존 CPU 비교 원본](../../experiments/logs/baseline_benchmark_report.md) · [4조건 주행 계획](../../paper/review/planning/2026-11-26_plan.md)

## 재사용할 자료

| 자료 | 확인한 근거 | JINT에서 쓸 범위 |
| --- | --- | --- |
| IEEE Access 저장소 원고 | `paper/workspace/IEEE_Access/main.tex`와 PDF 내용은 `post`와 현재 브랜치에서 동일하다. 원고 제목은 심사 메일의 제출번호 Access-2026-51740과 대응한다. 10월 9일 소스 ZIP은 Git 이력 `fdc3de4:paper/IEEE_Access_Source.zip`에 있다. | 투영식·가상 스캔 처리와 비교 방법을 참고했다. 실제 포털에서 받은 제출 패키지와의 파일 동일성은 미확인이다. |
| 분기별 구현 | 원격 `post`와 `backup/teammate-baseline`은 `31344e8`, 실험 브랜치는 `fce9756`이다. `98ca3bf`에서 깊이 비교가 수정됐고 `ebc011e`에서 바퀴 속도 피드백 중단 처리가 추가됐다. `main`·`rtabmap_ver`·11월 23일 PC 백업 bundle도 확인했다. | 현재 구현에 맞춰 방법을 작성했다. inflation 0.65 m와 실행 러너는 점검 당시 로컬 변경이다. 본 실험에서 실제 사용한 모델·LUT·설치본 및 파라미터를 기록한다. |
| 보정·학습 체크포인트 | `~/data/cam/usb_webcam.yaml`은 Git에 추출한 YAML과 동일하다. `best.pt` SHA-256은 `adad7307fd2a6e9ee17a565ccfb0138800129536959bfdf01186c5b485461886`이다. 학습 인수, 100 epoch 지표, OpenVINO export 메타데이터가 있다. | 마스크 AP50 0.802, AP50–95 0.562는 체크포인트에 저장된 집계값이다. 학습 데이터 실제 버전·이미지 수·분할·라벨 정의는 미확인이다. 모델에 표시된 이름과 배포 클래스 매핑이 달라 원본 `data.yaml`을 우선 찾는다. |
| 거리 LUT와 기록 좌표계 | 생성 코드로 LUT 3개를 다시 만들었고 저장 파일과 바이트 단위로 일치했다. 141개 명목 채널 중 카메라 열을 받는 채널은 127개이고, 중심선 근거리 지면 경계는 2.18 m이며 양쪽 가장자리의 방사 거리 경계는 약 2.56 m다. | 원고에 현재 설정과 모델상 한계를 기재했다. 파일럿 및 비교 bag 9개에서 기록된 `lidar_link`는 `base_link` 기준 x=-0.55 m였다. 이 값은 소프트웨어의 TF이며 물리 카메라 광학 원점과 구분한다. [LUT 감사 JSON](evidence/scan_lookup_geometry.json) · [기록된 TF CSV](evidence/scan_frame_transforms.csv) |
| CPU 비교 두 bag | `vlidar_run01/02`에는 RGB·가상 scan·odom이 있고, 이름이 `~/data/rosbags_22_trials.tar.gz` 목록에도 있다. 종전 CSV와 같은 200개의 원본 프레임으로 네 가지 구성을 재실행했다. 초기 프레임 2개 제외 후 CPU 시간 표본은 모델당 198개다. | PyTorch CPU의 V-LiDAR·MiDaS·Depth Anything과 배포용 OpenVINO의 처리 시간을 본문 표에 사용한다. 거리 오차는 실측 좌표계 확인 전까지 내부 진단 CSV에 보존한다. [재실행 요약 CSV](evidence/pytorch_comparison_summary.csv) |
| 9월 단일 장소 20회 | `minimal_A_scen1_run01…20`의 scan·odom과 스칼라 집계 CSV가 있다. 22회 백업 목록에도 해당 이름이 있다. RGB·전역 및 로컬 경로 메시지는 0개다. | 과거 궤적과 정지 징후를 분석할 수 있다. IEEE의 18/20과 IJCAS의 별도 구성(Stella VSLAM)에서 보고한 39/50은 새 주행 결과에 합산하지 않는다. |
| 11월 26일 RGB 주행 | 낮 복도 2개와 구름다리 주행 5개가 있다. 구름다리 01·02 성공 JSON은 `trial_id`가 비어 있어 파일명으로 연결한다. 휴지통에는 동명의 정지 녹화도 있다. | 01·02의 odom을 원본에서 읽고 출발 방향 기준 최대 횡변위 0.509/0.558 m를 재산출했다. 차체와 장애물 사이 간격은 실측이 필요하다. 03–05는 계획·odom 실패 분석에 쓴다. |
| 정지 두 구간의 시간 필터 비교 | 구름다리 정지 점검과 과거 광택 바닥 홀의 첫 12초에서 같은 RGB를 재생했다. 각 110프레임에서 필터 적용 시 유효·무효 변화 횟수는 8→0, 50→25였고, 공통 유효 채널의 프레임 간 평균 거리 변화는 약 41%·43% 감소했다. | 스캔 안정성의 보조 근거로 넣었다. 필터의 실제 주행 성공률 효과는 이 자료에서 평가하지 않는다. 구름다리 정지 점검 bag은 `~/data/jint_sources/bridge_static_20261126`에 하드링크로 보존했다. [결과 CSV](evidence/temporal_filter_replay.csv) |
| 파일럿 스캔 갱신률과 시각 기록 | 11월 26일 파일럿 7개의 스캔 5,048개 중 4,832개에서 같은 원본 시각의 RGB 프레임을 찾았다. 스캔 갱신률은 6.72–8.50 Hz였고, 구름다리 03의 95백분위 갱신 간격은 300 ms였다. | 실제 관측 갱신률의 근거로 사용한다. RGB보다 스캔이 먼저 저장된 프레임이 있는 두 bag의 시간차는 기록 순서를 보여준다. 모델 추론 벤치마크, 스캔 갱신률, Nav2 반응 시간을 따로 해석한다. [시각 분석 CSV](evidence/scan_recorder_timing.csv) |
| 과거 반사광·좁은 복도 영상 | 휴지통의 nav_debug_avoidance와 vlidar_run01.3/.4/.5를 SQLite로 직접 읽어 카메라 프레임 1,971·2,613·2,305·15,895개를 확인했다. 마지막 세 폴더는 metadata.yaml 없이 DB가 있다. | 의자, 역광·광택 바닥과 좁은 복도 사례를 재생할 수 있다. 설정·배치·외부 결과가 기록된 새 4조건 데이터와 구분한다. [기존 장면 분석 CSV](../../experiments/logs/glare_backlight_failure_audit.csv) |
| 다른 로봇 자료 | `~/bs_ws`에도 거리 실험이 있다. Waveshare UGV Beast PT·어안 카메라를 쓴 10–50 cm 실험이며 해당 bag은 LiDAR scan을 담고 있지 않다. LD19는 UGV의 장비 목록에 있다. | OMO-R1의 V-LiDAR와 비교할 동등 조건의 자료는 확인되지 않았다. |
| 원고·PC 스냅샷 | 별도 `ICCAS-IJCAS-2026` 저장소 `078bffc`, 로봇 PC의 `Downloads/2025-Capstone-main`, `backups/ros2_backup`과 팀원 백업을 검색했다. | 기존 그림·방법을 확인하는 데 쓴다. ICCAS/IJCAS의 공개·심사 상태와 JINT 원고의 추가 범위는 포털 기록과 공저자에게 확인한다. |

초안에는 11월 26일 CPU 비교 재실행 결과를 사용했다. 11월 24일 원본 CSV와 프레임 시각, 거리 참조, 방위각이 일치한다. 두 bag의 10월 2일 19:03·19:21 RGB에서 광택 있는 홀과 정지한 박스를 확인했다. 벤치마크의 장애물 좌표 `obs_x=4.4/4.7`은 코드에 있으나 실측 원점과 측정 원본은 확인하지 못했다. 같은 장애물 좌표를 odom 프레임에 고정한 채 기록된 x=-0.55 m를 적용하니 기존 59프레임의 참조 거리 자체가 0.51–0.55 m 늘고, 그중 47개가 원래의 거리·방위 필터에 남았다. 이 수치는 원점 가정에 대한 민감도다. 정지 2.50 m 실험은 원고·그림이 있고 원시 25프레임의 거리 값은 확인하지 못했다. [참조 원점 민감도 CSV](evidence/scan_origin_reference_sensitivity.csv)

## 남은 현장 작업

1. 수정된 odom으로 정지, 이전 목표 종료, 리셋, 다음 회차 시작을 실차에서 확인한다. 이 PC의 ROS 2 설치 환경에서 로드되는 MCU 모듈은 0.25초 피드백 타임아웃이 적용된 현재 소스를 가리킨다. 카메라 광학 중심의 높이·기울기와 실제 사용하는 스캔 원점을 첫 세팅 때 기록한다. 같은 설정으로 기존 장소 낮·밤, 새 인공조명 실내, 자연광 구름다리의 4조건을 취득한다.
2. [기존 현장 CSV 양식](../../paper/review/planning/templates/field_log.csv)에 각 시도의 실제 목표 구역 도달, 접촉, 수동 개입, bag ID, 실패 사유를 기록한다. 자료 이상으로 판정할 수 없는 시도도 행과 사유를 보존한다. 결과 집계에는 이 디렉터리의 summarize_trials.py에 작성한 현장 CSV를 넘긴다.
3. 성공과 실패를 대표하는 bag의 RGB·scan·경로·odom을 맞춰 지연 감지, 경로 생성, 이동 실행을 검토한다. 11월 26일 파일럿에서는 구름다리 01·02의 Nav2 완료, 03·04의 전역 경로 생성 실패, 05의 이전 목표 교체와 반복 속도 피드백을 ROS 로그와 원본 bag에서 구분했다. 새 주행에서 근거리 미검출이 반복되거나 독립 거리 결과를 쓰기로 한다면 해당 구간의 정적 클립을 추가한다. [Nav2 사건 CSV](evidence/pilot_nav_events.csv) · [속도 피드백 집계](evidence/pilot_trace_summary.csv)

11월 27일 로봇 PC의 로컬 통합 러너(`scripts/run_trial.sh`)에서 녹화 DB 생성 확인을 보완했다. 녹화 프로세스가 살아 있어도 10초 안에 DB 파일이 생기지 않으면 목표 전송 단계 전에 종료한다. DB 생성·미생성 분기는 모의 테스트로 확인했다. 첫 실차 파일럿에서는 저장된 bag의 RGB·scan·odom 메시지를 확인한다.

초안은 구조화된 실내에서 정적 장애물을 피하는 범위로 썼다. 선행연구에서도 단안 분할 기반 가상 LiDAR를 다뤘으므로, 이번 원고는 Nav2가 쓰는 거리·방위 관측과 실제 회피 결과를 연결하는 데 초점을 둔다. [유사 선행연구](https://www.fujipress.jp/jrm/rb/robot003200061137/)

## 수치 보충 자료

[Online Resource 2](evidence/ESM_2.zip)에는 CPU 프레임별 처리 시간, RGB·스캔 상대 시각, 필터 전후 스캔 배열이 있다. 압축을 푼 뒤 NumPy 환경에서 `python3 recompute_numeric_supplement.py`를 실행하면 세 표의 수치를 다시 계산한다. 원본 rosbag, RGB 영상, 본 주행의 현장 판정은 별도 자료다.

현재 PC의 원본에서 이 ZIP을 다시 만들 때는 `JINT/analysis/`에서 아래 명령을 실행한다. 투고할 때 [그림 `ESM_1.pdf`](evidence/ESM_1.pdf)와 [수치 `ESM_2.zip`](evidence/ESM_2.zip)을 원고 소스 ZIP과 별도의 파일로 올린다. [JINT 보충 자료 지침](https://link.springer.com/journal/10846/submission-guidelines#Supplementary%20Information%20(SI))

```bash
source /opt/ros/humble/setup.bash
OPENBLAS_NUM_THREADS=4 OMP_NUM_THREADS=4 python3 build_numeric_supplement.py
```

## 빌드 및 투고 전 확인

- 공식 양식 ZIP의 SHA-256은 `812e76dcaa9c28dc1bff1fb6065d51729b67d4ea140552a05088317414a3ecae`다. ZIP의 `sn-jnl.cls`와 번호 인용용 `sn-mathphys-num.bst`를 사용했다.
- 첫 페이지에 분야 번호, 150–250단어 영문 초록, 키워드 6개를 넣었다. 원고의 `Pending`은 본 실험 수치·최종 설정·학습 출처 및 분할·저자 선언·자료 공개 범위를 확인할 위치다. 저자 순서와 교신저자는 IEEE 원고에서 가져왔다. 초안 작성에 사용한 언어 모델은 Methods에 명시했다. [JINT 투고 지침](https://link.springer.com/journal/10846/submission-guidelines)
- ICCAS 공식 프로그램 FrBT3.1에 같은 제목의 확장초록과 39/50 결과가 공개되어 있다. 원고와 커버레터에 인용하고, 이번 연구에서 추가한 방법·비교·4조건 실차 평가를 명시했다. ICCAS의 IJCAS 연계 안내에서 proceedings와 IEEE Xplore 등재는 해당 원고의 처리 결과 및 저자 선택에 따라 달라진다. 이 초록의 별도 proceedings·DOI, IJCAS 심사 상태, 공저자 승인, 게재료와 영상 공유 범위는 투고 전 확인해야 한다. [ICCAS 프로그램](https://ras.papercept.net/conferences/conferences/ICCAS2026/program/ICCAS2026_ContentListWeb_4.html#frbt3_01) · [ICCAS–IJCAS 연계 안내](https://2026.iccas.org/authors-guideline-for-initial-submission/) · [JINT 가이드](https://link.springer.com/journal/10846/submission-guidelines)

아래 명령은 `JINT/analysis/`에서 실행한다. 투고 형식 초안은 `JINT/`, 주석이 있는 내부 검토본은 `JINT/planning/`, 소스 ZIP은 `JINT/manuscript/`에 생성된다. 소스 ZIP은 별도 디렉터리에서 컴파일을 확인한 뒤 저장한다.

```bash
bash ../manuscript/build_manuscript.sh
```

보정 모델의 관측 범위 그림, RGB를 연결한 구름다리 파일럿 궤적, 보충 자료 PDF는 ROS 2 환경에서 원본으로부터 다시 만들 수 있다. 휴지통 기록을 다른 위치에 보관할 경우 `--archive`로 지정한다.

```bash
source /opt/ros/humble/setup.bash
python3 render_evidence_figures.py
```

같은 ROS 2 환경에서 파일럿 진단 표와 그림을 재생성할 수 있다. CSV 시간은 bag 녹화 시작 기준이며 그림 하단의 scan 거리는 중앙 각도 창에서 가장 가까운 유효 반환이다.

```bash
python3 analyze_navigation_bags.py --output-dir evidence \
  --plot ../manuscript/figures/pilot_command_odometry.pdf --plot-bags nav_bridge_02 nav_bridge_05 \
  ~/data/bags/nav_bridge_0{1,2,3,4,5} \
  ~/data/bags/nav_test_11_26 ~/data/bags/nav_trail_1_26
```

스캔 갱신률과 기록 시각의 차이도 같은 bag에서 다시 구할 수 있다. 과거 CPU 비교용 bag 두 개는 원본 시각이 일치하는 비교 쌍이 있는지 확인하는 데만 더했다.

```bash
python3 scan_recorder_timing.py --output evidence/scan_recorder_timing.csv \
  ~/data/bags/nav_bridge_0{1,2,3,4,5} \
  ~/data/bags/nav_test_11_26 ~/data/bags/nav_trail_1_26 \
  ~/data/bags/vlidar_run0{1,2}
```

같은 CPU의 모델 비교는 두 bag의 원본 RGB를 다시 읽는다. 설치되어 있던 MiDaS·Depth Anything 모델 캐시를 사용했고 PyTorch 스레드를 4개로 고정했다. 모델별 기본 입력 크기와 실행 환경은 별도로 저장했다. [실행 환경](evidence/pytorch_comparison_environment.json) · [입력·모델 해시](evidence/pytorch_comparison_inputs.sha256)

```bash
HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 OPENBLAS_NUM_THREADS=4 \
OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 python3 bench_jint_cpu.py \
  --output-dir evidence --threads 4 \
  ~/data/bags/vlidar_run01/vlidar_run01_0.db3 \
  ~/data/bags/vlidar_run02/vlidar_run02_0.db3
```

현재 LUT 파일과 bag 시작 3초에 기록된 센서 프레임을 다시 확인하는 명령이다. 선택적으로 위 CPU 벤치마크 출력에서 참조 거리의 원점 민감도도 계산한다.

```bash
python3 audit_scan_geometry.py --output-dir evidence --comparison-dir evidence \
  --calibration ~/data/cam/usb_webcam.yaml \
  ~/data/bags/nav_bridge_0{1,2,3,4,5} \
  ~/data/bags/nav_test_11_26 ~/data/bags/nav_trail_1_26 \
  ~/data/bags/vlidar_run0{1,2}
```

### 배포 클래스 검증

현재 `best.pt`의 SHA-256은 `603784e`(2025-02-06)에 올라온 가중치와 같다. 체크포인트의 학습 인수는 `data.yaml`, `train3`을 가리키며, 체크포인트와 OpenVINO 내보내기 메타데이터에는 `0: person`, `1: floor`로 표시된다. 이전 IJCAS 원고의 Roboflow 링크는 현재 `Floor` 한 클래스를 표시한다. 공개 버전의 이미지 수나 분할과 현재 체크포인트 사이의 연결은 확인되지 않았다. [체크포인트 메타데이터](../../experiments/logs/pc_extracted/yolo11n_seg_checkpoint_meta.json) · [OpenVINO 내보내기 정보](../../src/freespace_detection/scripts/best_openvino_model/metadata.yaml) · [IJCAS 원고 대조](../../paper/review/10_IJCAS_Manuscript_Repository_Check_20261118.md) · [Roboflow 프로젝트](https://universe.roboflow.com/projects-aw2cm/floor-segmentation-wesrg)

실제 배포에서 사용한 ID는 RGB와 `/scan`의 획득 타임스탬프가 같은 쌍을 OpenVINO로 재생해 확인했다. 마스크를 고르는 ID만 바꾸고 투영·필터·출력 순서를 같게 했다. 첫 1초를 제외한 뒤 141개 채널 전체가 기록값과 일치한 스캔을 셌다. 허용 차이는 0.1 mm이며 한 쌍의 양의 무한대는 일치로 셌다. ID 0은 낮 복도 1,055/1,057개, 구름다리 390/423개가 일치했다. ID 1은 두 기록 모두 전체 일치가 0개였다. 이 대조는 기록 스캔을 재현하는 클래스 설정을 확인한다. 학습 라벨과 실제 바닥 분할의 픽셀 단위 정확도는 별도 근거가 필요하다. [클래스 검증 CSV](evidence/floor_class_scan_replay.csv) · [입력 해시](evidence/floor_class_scan_inputs.sha256)

```bash
OPENBLAS_NUM_THREADS=4 OMP_NUM_THREADS=4 python3 audit_floor_class.py \
  --output evidence/floor_class_scan_replay.csv \
  ~/data/bags/nav_test_11_26 ~/data/bags/nav_bridge_02
```

### 시간 필터 재생 실험

같은 환경에서 카메라 원본으로 시간 필터 비교표를 재현할 수 있다. 스크립트는 ROS 노드를 시작하지 않고, 모델·마스크·LUT를 공유하는 경로에서 필터 전후 거리를 모은다. 원본과 코드의 해시 목록은 저장소 루트 기준이다. [입력 해시 목록](evidence/temporal_filter_inputs.sha256)

```bash
OPENBLAS_NUM_THREADS=4 OMP_NUM_THREADS=4 python3 replay_scan_filter.py \
  --seconds 12 --output evidence/temporal_filter_replay.csv \
  ~/data/jint_sources/bridge_static_20261126 ~/data/bags/vlidar_run01
```

이동 구간은 `Begin navigating`부터 `Goal succeeded`까지다. 이전 1초로 필터를 초기화했다. 기록된 `/scan`과 획득 타임스탬프가 같은 RGB만 사용해 당시의 영상 간격을 반영했다. 현재 모델과 LUT로 두 변형을 재생한 값이다. 최초 유효 반환이 활성화 버전에서 보류되고 바로 다음 프레임에서 두 변형 모두 유효한 경우, 두 RGB 획득 시각의 차이를 확인 지연으로 집계했다. 유효 반환의 변화와 지연은 각 각도 채널에서 측정했다. 이 자료에는 장애물별 정답 라벨이 없다. [구름다리 01](evidence/temporal_filter_bridge_01.csv) · [구름다리 02](evidence/temporal_filter_bridge_02.csv) · [구간 근거 로그](evidence/pilot_nav_events.csv)

```bash
export OPENBLAS_NUM_THREADS=4 OMP_NUM_THREADS=4
python3 replay_scan_filter.py --scan-stamped --start 11.078 --seconds 62.062 \
  --output evidence/temporal_filter_bridge_01.csv ~/data/bags/nav_bridge_01
python3 replay_scan_filter.py --scan-stamped --start 6.296 --seconds 45.940 \
  --output evidence/temporal_filter_bridge_02.csv ~/data/bags/nav_bridge_02
```

## 실험 환경 사진과 기존 원고 그림

현재 환경 사진은 `vlidar_run01`의 15초 이후 첫 RGB 프레임(반사 바닥과 박스), `nav_bridge_02`의 5초 이후 첫 RGB 프레임(구름다리와 통)이다. 모두 bag 시작 시각 기준이며 스크립트가 사용한 실제 프레임 시각을 출력한다. 이 두 장은 이전 기록의 환경과 장애물을 보여준다. [실험 환경 그림](../manuscript/figures/reference_sites.pdf)

카메라·costmap 그림은 기존 회피 시퀀스의 가운데 화면에서 두 창을 잘라 냈다. 원고의 반사 사례는 box 주변을 잘라 floor mask와 기존 원 표시를 남겼다. 두 원본은 IEEE Access와 IJCAS 소스에 같은 파일로 들어 있다. 출판 상태와 재사용 조건을 교신저자가 제출 전에 확인한다. [기존 회피 화면](../../paper/workspace/IEEE_Access/figures/nav2_obstacle_avoidance_sequence.png) · [기존 반사 예시](../../paper/workspace/IEEE_Access/figures/segmentation_artifact.png)

### 추가 그림의 출처

Figure 1은 ICCAS 확장초록의 전체 구성도를 참고해 인식·가상 스캔·Nav2·차체 제어와 odom 피드백을 그렸다. 예시 RGB는 `nav_bridge_02`의 19.052 s 프레임(기록 시각 `1795676540151140856` ns)이다. OpenVINO 재생으로 바닥 마스크를 만들었고, `/scan`은 RGB와 획득 시각이 같은 기록을 사용했다. 코스트맵·로컬 경로·odom은 RGB 기록 시각 기준으로 각각 +44.3, −30.3, −3.2 ms의 메시지를 골랐다. 로봇 위치 변환에는 bag의 TF를 적용했다. [ICCAS 확장초록](../../IJCAS/ICCAS/ICCAS_Extended_Abstract.pdf)

```bash
source /opt/ros/humble/setup.bash
python3 render_pipeline_figure.py ~/data/bags/nav_bridge_02 19
```

| 그림 | 출처와 해석 범위 |
| --- | --- |
| 본문 관측 범위 | IEEE 원고의 근거리 구조 설명을 바탕으로 [보정 및 LUT 검증값](evidence/scan_lookup_geometry.json)을 가지고 다시 그렸다. 중앙선의 2.18 m와 ±31.5°의 127개 채널은 저장된 보정 모델의 기하 한계다. |
| 본문 구름다리 우회 | `nav_bridge_01/02`의 `/odom`을 출발 자세에 맞췄다. `nav_bridge_02`에서 녹화 시작 후 10.010·25.017·35.006 s의 RGB와 가장 가까운 odom을 연결했다. 기록 시각 차이는 각각 6.2·5.1·15.8 ms이며 센서 획득 지연 측정값과 구분한다. |
| 보충 그림 S1 | rosbag `/camera/image_raw`의 기록 시각 기준. `(a) nav_debug_avoidance` 1789917441448965025 ns, `(b) vlidar_run01.3` 1790854533842965842 ns, `(c) vlidar_run01.4` 1790932912384048917 ns, `(d) vlidar_run01.5` 1790933826258576115 ns. 의자·카트·역광·좁은 복도의 대표 장면이다. |
| 보충 그림 S2 | [비교 run01](evidence/pytorch_comparison_run01.csv)과 [run02](evidence/pytorch_comparison_run02.csv)의 CPU 처리 시간에서 첫 프레임을 한 장씩 제외했다. 구현당 198개 값의 경험적 누적 분포다. |
| 보충 그림 S3 | [9월 20회 원본 점검 JSON](../../paper/review/verification/robot_20261118/twenty_runs.json)의 최종 odom x와 녹화 말미 정지 시간이다. 외부 도달·접촉 판정은 확인되지 않았다. |
| 보충 그림 S4 | `nav_bridge_05`의 RGB 2.959·9.291·14.017 s와 가장 가까운 `/odom`을 대조했다. 기록 시각 차이는 각각 +6.9·−6.9·−12.5 ms(odom−RGB)다. 첫 프레임부터 마지막 프레임까지 odom의 누적 yaw 변화는 −210.60°였고 영상 내 대응점 이동의 중앙값은 0.18 px였다. |

S4에서는 첫 640×480 RGB 프레임에서 OpenCV로 코너를 최대 250개 검출하고 마지막 프레임까지 Lucas–Kanade로 추적했다. 양방향 추적이 성공하고 원래 지점까지의 복귀 오차가 1 px 미만인 247개를 집계했다. 카메라 영상은 odom 이상을 확인하는 정성적 근거이며, 차체 자세의 실측값이나 피드백이 정상일 때의 도달 여부는 이 기록으로 구할 수 없다. [S4 재생성 코드](render_evidence_figures.py)

원고에는 그림 S1–S4를 Online Resource 1로 인용했다. 투고 시 [보충 자료 `ESM_1.pdf`](evidence/ESM_1.pdf)를 LaTeX 소스 ZIP과 별도 파일로 올린다. 기존 원고에 있는 정지 2.50 m 거리 그래프는 독립 실측 원점과 원본 프레임별 값이 확인되면 거리 정확도의 근거로 재검토할 수 있다. [기존 거리 그림](../../paper/workspace/IEEE_Access/figures/experiment1_results.png) · [좌표 원점 민감도](evidence/scan_origin_reference_sensitivity.csv)

`JINT/analysis/`에서 아래 명령으로 세 그림을 재생성할 수 있다.

```bash
source /opt/ros/humble/setup.bash
python3 render_archive_figures.py \
  --frame ~/data/bags/vlidar_run01 15 \
  --frame ~/data/bags/nav_bridge_02 5 \
  --output ../manuscript/figures/reference_sites.pdf --archive-views
```

본 실험을 마치면 같은 스크립트에 장애물이 보이는 RGB 프레임을 지정한다. 아래 경로와 초는 현장 bag에 맞게 바꾸고, **기존 장소 낮 → 기존 장소 밤 → 인공조명 실내 → 자연광 구름다리** 순서를 유지한다. `formal_sites.pdf`가 있으면 빌드 시 본문에 해당 4패널이 표시된다. 대표 프레임의 bag ID와 시각을 현장 로그에 남긴다.

```bash
python3 render_archive_figures.py \
  --frame "$bag_original_day" "$time_original_day" \
  --frame "$bag_original_night" "$time_original_night" \
  --frame "$bag_indoor_artificial" "$time_indoor_artificial" \
  --frame "$bag_bridge_daylight" "$time_bridge_daylight" \
  --output ../manuscript/figures/formal_sites.pdf
bash ../manuscript/build_manuscript.sh ~/data/jint/field_log.csv
```

### 대표 주행의 RGB와 궤적

현장 CSV의 `record_id`를 지정하면 `bag_asset_id`의 녹화에서 RGB 세 장과 `/odom` 궤적을 그린다. 장애물이 보이는 접근, 우회, 통과 구간의 시각을 초 단위로 지정한다. 실패 사례는 접근과 정지 등 실패 지점을 설명하는 세 장을 고른다. 타임스탬프는 녹화 시작 기준이고, 도달·실패 표시는 집계 코드와 같은 현장 CSV 판정을 사용한다.

```bash
source /opt/ros/humble/setup.bash
python3 render_trial_sequence.py ~/data/jint/field_log.csv "$record_id" \
  --frames "$first_s" "$middle_s" "$last_s" \
  --output ../manuscript/figures/formal_trial_sequence.pdf
bash ../manuscript/build_manuscript.sh ~/data/jint/field_log.csv
```

다른 대표 사례는 `formal_trial_sequence_2.pdf`로 추가하면 본문에 함께 들어간다. 집계표와 같은 설정의 회차를 고른다. CSV에 목표 수락·종료 시각이 있으면 그 구간을 그린다. 시각이 없는 기록에는 `--window START STOP`으로 범위를 지정할 수 있다. 스크립트 출력에 RGB와 대응 `/odom`의 정확한 기록 시각과 차이가 남는다. [대표 주행 그림 코드](render_trial_sequence.py)

## 현장 CSV의 자동 기록과 집계

첫 파일럿이나 장비·녹화 설정을 바꾼 직후, 녹화가 끝난 bag에서 RGB·`/scan`·`/odom` 표본이 실제로 열리는지 확인한다. 아래 명령은 각 토픽의 첫 메시지를 SQLite에서 읽어 디코딩하고, 명령·TF·경로·코스트맵의 기록 건수를 같이 표시한다. `--result`와 `--trial-id`를 함께 주면 결과 JSON에 같은 회차 ID가 저장됐는지도 확인한다. 진단 결과와 현장 성공 판정은 별도로 기록한다.

```bash
source /opt/ros/humble/setup.bash
python3 trial_records.py check \
  --bag ~/data/bags/nav_original_day_s1_01 \
  --result ~/data/trial_outcomes/original_day_s1_01.json \
  --trial-id original_day_s1_01
```

실험에 사용할 설정과 파일을 정했을 때 한 번 저장한다. 해시는 현재 PC의 파일에 대한 기록이다. 실차에서 로드된 런치 인수와 유효 파라미터는 별도로 현장 기록에서 확인한다. [현재 PC 파일 스냅샷](evidence/robot_checkout_snapshot_2026-11-26.json)

```bash
python3 trial_records.py snapshot \
  --output ~/data/jint/field_config_01.json \
  --camera-calibration ~/data/cam/usb_webcam.yaml
```

주행 직후 기존 러너의 세션 이름을 회차 ID로 사용해 다음처럼 등록한다. 이 명령은 bag과 JSON을 읽고 현장 CSV에만 기록한다. 예시의 환경 ID와 배치 ID를 해당 조건에 맞춰 바꾼다. [실제 파일럿에서 확인한 CSV](evidence/pilot_field_log.csv)

```bash
python3 trial_records.py add \
  --field-log ~/data/jint/field_log.csv --trial-id original_day_s1_01 \
  --phase main --environment original_day --layout S1 \
  --configuration ~/data/jint/field_config_01.json \
  --bag ~/data/bags/nav_original_day_s1_01 \
  --result ~/data/trial_outcomes/original_day_s1_01.json
```

결과 CSV에서 외부 목표 도달·접촉·개입과 최종 결과를 확인해 채운다. 성공 조건을 충족한 행은 `outcome=success`, `contact_observed=false`, `manual_intervention=false`, `evaluation_included=true`를 지정하고 `external_goal_check`에 관찰 근거를 남긴다. JSON의 Nav2 상태는 별도로 저장되고, 실패로 산출되지 않은 파일 인수는 생략해도 시도를 기록할 수 있다.

새 Nav2 결과 JSON에는 클라이언트가 목표 수락과 첫 종료 사건을 관측한 ROS 시각을 기록한다. 타임아웃에서는 취소 요청을 시작한 시각을 쓰며, 뒤늦게 도착한 취소 응답은 기존 타임아웃을 덮어쓰지 않는다. 두 시각이 유효한 경우 CSV에 경과 시간이 자동으로 들어간다. 과거 JSON에 없던 시각은 공란으로 남는다. [주행 결과 JSON 작성 코드](../../scripts/test_avoidance_goal.py) · [현장 CSV 가져오기](trial_records.py)

집계표는 설정·조건·배치별로 성공, 실패, 무효, 판정 보류를 표시한다. 다른 설정으로 바꿔 취득한 회차는 설정별 표에서 확인할 수 있다.

```bash
python3 summarize_trials.py ~/data/jint/field_log.csv
```

논문 결과 표에는 본 주행 CSV의 확인된 결과를 바로 넣을 수 있다. 기본 환경 ID 순서는 `original_day`, `original_night`, `indoor_artificial`, `bridge_daylight`다. 배치 ID는 박스가 `S1`, 박스와 의자가 `S2`다. 현장에서 다른 ID를 사용했다면 `--condition-ids`에 환경 ID 4개, `--layout-ids`에 배치 ID 2개를 이 순서로 넘긴다.

```bash
bash ../manuscript/build_manuscript.sh ~/data/jint/field_log.csv
```

CSV에 본 주행 설정이 여러 개 있으면 `--configuration`으로 설정 ID를 지정한다. 빌드가 표와 성공률 그래프에 같은 설정·환경·배치·분모를 적용한다. 시도 수에는 무효와 판정 보류가 남고, 성공률과 Wilson 구간은 판정 가능한 시도 기준으로 계산한다. 그래프의 `n/a`는 미기록 조합이고, `0 eval.`은 시도가 기록됐으나 판정 가능한 시도가 없는 조합이다.
