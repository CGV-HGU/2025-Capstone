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
- ICCAS FrBT3.1은 2026년 10월 30일 공식 프로그램으로 인용했다. 학회에는 11월 10일자 proceedings 배포본이 표시되며, 개별 수록 파일과 DOI는 미확인이다. IEEE Access 거절 통지와 IJCAS 보관 원고를 대조해 공유 내용과 JINT의 재분석을 커버레터에 정리했다. [ICCAS 프로그램](https://ras.papercept.net/conferences/conferences/ICCAS2026/program/ICCAS2026_ContentListWeb_4.html#frbt3_01) · [Proceedings 페이지](https://sigongji.2026.iccas.org/Proceedings/HomeIF.asp) · [커버레터의 근거](../planning/cover_letter_draft.md#공저자-확인용)

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

### 기준 장소와 낮 주행 기록

기존 IEEE Access와 IJCAS 원고의 Experiment 2 세팅 사진은 같은 JPEG이며, EXIF 촬영 시각은 2026년 5월 27일 17:22:53 KST다. `~/data/db/video/NTH4F.mp4`의 약 70초에서 같은 ‘ARISE & SHINE’ 표지와 출입문이, 약 100초에서 벽돌 계단실과 둥근 기둥이 보인다. 4조건 본 주행의 낮·밤 기준 장소는 이 사진의 **뉴턴홀 4층 열린 홀**로 잡는다. EXIF 시각은 세팅 사진의 시각이며, 기존 주행 회차의 시각은 따로 확인해야 한다. [기존 세팅 사진](../../IJCAS/ICCAS/IJCAS/paper/experiment2_setup.jpeg) · [360° 영상 인벤토리](../../paper/review/04_Robot_PC_Asset_Audit_and_Log_Inventory.md)

| 보관 기록 | 확인한 위치 및 활용 범위 |
| --- | --- |
| `nav_trail_1_26` · 11월 26일 14:54 KST | Bag의 약 37초 RGB와 `~/data/db/video/OH3F.mp4`의 약 50초에 같은 문 장식, 창틀, 맞은편 유리벽이 보인다. **오석관 3층 유리벽 복도**이며, 인공조명 위주 실내 조건 후보의 파일럿으로 쓴다. 본 주행 때 조명 환경과 실제 도달·접촉 여부를 기록한다. |
| `nav_test_11_26` · 11월 26일 13:50 KST | 약 57초 RGB에서 계단 입구와 목재 사물함이 있는 좁은 복도가 보인다. 기준 사진의 열린 홀과 배치가 다르며 정확한 건물과 층은 미확인이다. |
| `minimal_A_scen1_run01…20` · 9월 21일 오전 | 활성 폴더의 20회에 scan·odom이 있다. RGB가 없어 기준 홀의 배치를 화면으로 대조할 수 없다. 같은 이름의 휴지통 `run01`은 전날 23:59 KST에 시작한 별개 녹화다. |
| `nav_debug_avoidance`와 `vlidar_run01/02` · 9–10월 야간 | 의자 또는 박스가 있는 반사 바닥 홀의 RGB와 scan을 보존했다. 기존 사진과 동일한 10 m 배치였는지는 미확인이다. |

보관 자료에서 뉴턴홀 4층의 낮 주행으로 대조 가능한 RGB rosbag은 확인되지 않았다. 관찰 기록과 bag ID를 연결할 수 있게 기준 홀의 낮·밤을 같은 배치로 취득한다. [PC rosbag 목록](evidence/rosbag_inventory.csv)

### 원고에 반영한 장면

기존 환경 사진 표본은 `vlidar_run01`의 15초 이후 첫 RGB 프레임(반사 바닥과 박스)과 `nav_bridge_02`의 5초 이후 첫 RGB 프레임(구름다리와 통)이다. 아래 본 실험 환경 그림의 프레이밍을 잡는 데 참고할 수 있다. [파일럿 RGB 표본](../manuscript/figures/reference_sites.pdf)

카메라·costmap 그림은 `nav_trail_1_26`의 37.062 s RGB(`1795672534045552227` ns)로 다시 그렸다. `/scan`은 RGB와 획득 시각이 같고, `/local_costmap/costmap`, `/local_plan`, `/odom`은 RGB의 기록 시각 기준으로 각각 +61.2, +36.1, −3.3 ms다. 로봇 중심의 좌표 변환은 같은 bag의 TF를 사용했다. 반사 바닥 그림은 `vlidar_run01.4`의 5.013 s RGB(`1790932901883577839` ns)를 현재 OpenVINO 모델로 재생했다. 같은 검출 결과에 하단 밴드 조건을 적용했을 때 영상 위쪽의 마스크 1,421픽셀(320×256 모델 격자)이 빠졌다. 앞서 사용한 회피 화면과 반사 예시는 IEEE Access와 IJCAS 소스에서 확인할 수 있다. [새 복도 그림](../manuscript/figures/archive_nav2_view.pdf) · [새 마스크 그림](../manuscript/figures/archive_floor_reflection.pdf) · [기존 회피 화면](../../paper/workspace/IEEE_Access/figures/nav2_obstacle_avoidance_sequence.png) · [기존 반사 예시](../../paper/workspace/IEEE_Access/figures/segmentation_artifact.png)

### 추가 그림의 출처

Figure 1은 ICCAS 확장초록의 전체 구성도를 참고해 RGB·바닥 마스크와 경계·가상 스캔·코스트맵을 같은 크기의 패널로 연결했다. 아래에는 목표, Nav2 제어기, 로봇, odom 피드백을 배치했다. 예시 RGB는 `nav_bridge_02`의 19.052 s 프레임(기록 시각 `1795676540151140856` ns)이다. OpenVINO 재생으로 바닥 마스크를 만들었고, `/scan`은 RGB와 획득 시각이 같은 기록을 사용했다. 코스트맵·로컬 경로·odom은 RGB 기록 시각 기준으로 각각 +44.3, −30.3, −3.2 ms의 메시지를 골랐다. 로봇 위치 변환에는 bag의 TF를 적용했다. [ICCAS 확장초록](../../IJCAS/ICCAS/ICCAS_Extended_Abstract.pdf) · [JINT Figure 1](../manuscript/figures/pipeline.pdf)

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
| 보충 그림 S5 | `nav_bridge_04`의 두 번째 전역 계획 실패(93.798 s) 앞뒤 `/global_costmap/costmap`과 다음 `/plan`을 대조했다. 목표를 중심으로 x·y 각각 ±0.5 m 안에 중심이 있는 100칸을 집계했다. 실패 274 ms 전에는 99 또는 100인 칸이 96개였고, 초기화 요청 후 26 ms 시점 지도에서는 100칸 모두 0이었다. |
| 보충 그림 S6 | `nav_bridge_02`의 19.052 s RGB와 획득 시각이 같은 가상 스캔에서 통 하단을 향하는 5개 채널(0 기준 52–56)의 투영점 중앙값을 기준으로 삼았다. 기록된 TF와 획득 시각이 가장 가까운 odom을 써서 지름 0.5 m의 고정 영역에서 스캔 반환과 지도 셀을 집계했다. |

S4에서는 첫 640×480 RGB 프레임에서 OpenCV로 코너를 최대 250개 검출하고 마지막 프레임까지 Lucas–Kanade로 추적했다. 양방향 추적이 성공하고 원래 지점까지의 복귀 오차가 1 px 미만인 247개를 집계했다. 카메라 영상은 odom 이상을 확인하는 정성적 근거이며, 차체 자세의 실측값이나 피드백이 정상일 때의 도달 여부는 이 기록으로 구할 수 없다. [S4 재생성 코드](render_evidence_figures.py)

계획 실패 다섯 건 모두 초기화 요청과 같은 목표를 향하는 전역 경로가 91 ms 이내에 다시 기록됐다. S5에서는 실패 약 19 ms 뒤에 초기화 요청, 약 29 ms 뒤에 다음 경로가 저장됐다. 다른 네 실패의 직전 지도는 1.74–15.63 s 앞선 표본이다. 구름다리에서 어떤 물리 물체나 센서 반환이 높은 비용을 만들었는지는 확인되지 않았다. Nav2가 내보낸 `OccupancyGrid`에서 99는 차량의 내접 반경에 해당하는 장애물 비용이고 100은 장애물 셀이다. [실패 전후 집계](evidence/pilot_planner_costmaps.csv) · [재생성 코드](audit_planner_costmaps.py) · [Nav2 비용 변환 공식 소스](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/costmap_2d_publisher.cpp#L81-L93)

S6의 시간축은 rosbag 기록 시각이다. 영역 내 스캔 반환의 마지막 기록은 22.336 s이고, 비용 100인 셀은 22.497 s 지도에 마지막으로 남았다. 24.697 s 지도에도 비용 99인 셀이 두 개 있었다. 23.016 s RGB에서는 통의 바닥 접점이 프레임 밑으로 잘려 있다. 설정의 최소 ray-clearing 거리는 관측 레이의 삭제 시작점을 제한한다. 기록된 지도에는 팽창 비용도 포함돼 이 구간에서 두 효과가 각각 얼마나 기여했는지는 알 수 없다. [스캔·costmap 시계열](evidence/pilot_near_field_trace.csv) · [재생성 코드](audit_near_field_costmap.py) · [Nav2의 ray-clearing 구현](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/include/nav2_costmap_2d/costmap_2d.hpp#L389-L423)

원고에는 그림 S1–S6을 Online Resource 1로 인용했다. 투고 시 [보충 자료 `ESM_1.pdf`](evidence/ESM_1.pdf)를 LaTeX 소스 ZIP과 별도 파일로 올린다. 기존 원고에 있는 정지 2.50 m 거리 그래프는 독립 실측 원점과 원본 프레임별 값이 확인되면 거리 정확도의 근거로 재검토할 수 있다. [기존 거리 그림](../../paper/workspace/IEEE_Access/figures/experiment1_results.png) · [좌표 원점 민감도](evidence/scan_origin_reference_sensitivity.csv)

`JINT/analysis/`에서 아래 명령으로 세 그림을 재생성할 수 있다.

```bash
source /opt/ros/humble/setup.bash
python3 render_archive_figures.py \
  --frame ~/data/bags/vlidar_run01 15 \
  --frame ~/data/bags/nav_bridge_02 5 \
  --output ../manuscript/figures/reference_sites.pdf
OPENBLAS_NUM_THREADS=4 OMP_NUM_THREADS=4 python3 render_navigation_examples.py
```

초안의 실험 환경 그림은 같은 크기의 4:3 TBD 박스 네 개를 2×2로 배치했다. 본 실험을 마치면 같은 스크립트에 RGB 프레임을 **기준 장소 낮 → 기준 장소 밤 → 인공조명 실내 → 자연광 구름다리** 순서로 지정한다. 네 조건을 비교하기 좋게 같은 박스 배치(`S1`)의 주행에서 우회 직전 박스와 바닥이 함께 보이는 전방 프레임을 우선 고른다. 대표 프레임의 bag ID와 시각을 현장 로그에 남긴다. `formal_sites.pdf`가 있으면 빌드에서 사진이 같은 위치의 박스를 대체한다. [TBD 배치 PDF](../manuscript/figures/formal_sites_tbd.pdf)

```bash
python3 render_archive_figures.py --placeholder --output ../manuscript/figures/formal_sites_tbd.pdf
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

## 본 주행 저장 공간

2026-11-27 로봇 PC의 `~/data/bags` 가용 공간은 **127.1 GB**였다. 11월 26일 구름다리 5개와 복도 2개의 실제 압축 파일과 압축을 풀었을 때의 DB 크기를 쟀다. 회당 압축본은 **0.85–2.08 GB**, 원본 DB는 **1.18–3.50 GB**였다. 현재 러너의 파일 단위 Zstandard 방식에서는 압축본을 쓴 뒤 원본 DB를 지우므로 종료 처리 중 잠시 두 파일이 공존한다. 아래 피크는 한 번에 한 회차를 녹화하고 압축 완료를 기다리는 경우다. 외부 영상, 별도 분석 중인 DB, 다른 프로세스 사용량은 별도로 잡는다. [파일별 측정 CSV](evidence/pilot_bag_storage.csv) · [rosbag2 파일 압축 소스](https://github.com/ros2/rosbag2/blob/humble/rosbag2_compression/src/rosbag2_compression/sequential_compression_writer.cpp#L263-L295)

| 80회 녹화 가정 | 압축본 합계 | 압축 종료 시 예상 피크 |
| --- | ---: | ---: |
| 기존 7회의 평균 크기 | 123.4 GB | 126.9 GB |
| 기존 7회 중 가장 큰 bag 크기 | 166.1 GB | 169.6 GB |
| bag 전체 길이 120초, 7회 합산 저장률 | 167.2 GB | 170.5 GB |
| bag 전체 길이 120초, 최대 관측 저장률 | 189.9 GB | 193.3 GB |

현재 공간은 120초 bag을 조건별 20회 녹화하는 데 필요한 **약 45–51 GB**를 감당한다. 전체를 로봇 PC 한 곳에 모으면 80회 시점에 여유가 거의 없거나 공간이 부족할 수 있다. 조건별로 압축본과 판정 CSV·JSON을 묶어 보관할 수 있도록 별도 저장 위치를 확보하는 편이 좋다. 120초는 bag 전체 길이이며, 러너의 목표 제한 시간에는 녹화 시작 후 대기와 종료 처리 시간이 추가된다.

대표 bag 두 개의 SQLite `messages.data` 길이를 토픽별 합산했다. `/camera/image_raw`가 `nav_bridge_02`와 `nav_test_11_26`의 직렬화 메시지 용량에서 각각 **99.2%, 99.6%**를 차지했다. 두 기록의 action 메시지 용량은 0이었다. 이 비율은 압축 전 토픽 데이터 기준이다.

```bash
python3 audit_bag_storage.py --target ~/data/bags --trials 80 --recording-seconds 120 \
  ~/data/bags/nav_bridge_0{1,2,3,4,5} ~/data/bags/{nav_test_11_26,nav_trail_1_26}
```

### 조건별 백업

한 조건의 녹화와 현장 로그를 마친 뒤 실제 백업 장치의 경로를 `backup_root`로 지정한다. 아래 명령은 해당 조건의 CSV 행, 완료된 rosbag, 결과 JSON과 행에 연결된 설정 JSON을 함께 복사한다. 각 복사본의 SHA-256이 원본과 일치할 때 백업 폴더가 생성된다. 외부 관찰 영상은 현장 CSV의 영상 ID와 함께 별도 보관한다.

```bash
python3 archive_field_trials.py ~/data/jint/field_log.csv \
  --environment original_day --output "$backup_root/original_day"
(cd "$backup_root/original_day" && sha256sum -c SHA256SUMS)
```

백업의 `field_log.csv`는 포함된 결과와 설정의 상대 경로를 담는다. 궤적을 다시 그릴 때 이 CSV와 백업의 `bags/`를 `render_trial_sequence.py --bags`에 지정한다. 전송할 위치와 자료 접근 범위는 교신저자가 확인한다. [백업 스크립트](archive_field_trials.py)

## 현장 CSV의 자동 기록과 집계

첫 파일럿이나 장비·녹화 설정을 바꾼 직후, 녹화가 끝난 bag에서 RGB·`/scan`·`/odom` 표본이 실제로 열리는지 확인한다. 아래 명령은 각 토픽의 첫 메시지를 SQLite에서 읽어 디코딩하고, 명령·TF·경로·코스트맵의 기록 건수를 같이 표시한다. `--result`와 `--trial-id`를 함께 주면 결과 JSON에 같은 회차 ID가 저장됐는지도 확인한다. 진단 결과와 현장 성공 판정은 별도로 기록한다.

```bash
source /opt/ros/humble/setup.bash
python3 trial_records.py check \
  --bag ~/data/bags/nav_original_day_s1_01 \
  --result ~/data/trial_outcomes/original_day_s1_01.json \
  --trial-id original_day_s1_01
```

실험에 사용할 설정을 정하면 아래 명령으로 한 번 저장한다. `--installed`는 현재 ROS 환경이 찾는 파일 경로와 해시를 함께 남긴다. 11월 27일 PC에서는 MCU·패킷 수신 코드, 바닥 검출과 스캔 노드, Nav2 설정과 관련 런치의 11개 파일이 소스와 일치했다. `pose_converter`는 설치된 실행 파일의 해시도 따로 남겼다. 런치 인수와 실행 중에 실제 적용된 파라미터는 현장 기록에서 확인한다. [설치본 스냅샷](evidence/robot_install_snapshot_2026-11-27.json)

```bash
source /opt/ros/humble/setup.bash
source ../../install/setup.bash
python3 trial_records.py snapshot --installed \
  --output ~/data/jint/field_config_01.json \
  --camera-calibration ~/data/cam/usb_webcam.yaml \
  --lidar-x-offset -0.55
```

`--lidar-x-offset`에는 통합 러너에 전달할 실험값을 넣는다. 새 `configuration_id`는 기록한 백엔드·장치·오프셋과 파일 해시로 계산한다. 같은 파일과 설정을 다시 기록하면 문서 커밋이나 보정 파일 경로가 바뀌어도 같은 ID를 얻는다. 당시 Git 커밋과 파일 경로는 스냅샷에 따로 남으며, 기존 CSV와 스냅샷의 ID는 그대로 읽는다.

11월 파일럿 일곱 개의 `/tf`와 `/tf_static` 전체에서 `map → odom` 12,268개를 읽었고 모두 항등 변환이었다. `/odom`은 일관되게 `odom → base_footprint`를 기록했다. 설치된 Nav2 YAML은 AMCL의 TF 발행을 끄고 costmap에 장애물·팽창 레이어를 사용한다. 이 기록은 파일럿의 경로가 바퀴 odom 좌표를 따랐다는 근거다. [파일럿 좌표계 집계](evidence/pilot_odometry_frames.csv) · [집계 코드](audit_odometry_frame.py)

주행 직후 기존 러너의 세션 이름을 회차 ID로 사용해 다음처럼 등록한다. 이 명령은 bag과 JSON을 읽고 현장 CSV에만 기록한다. 예시의 환경 ID와 배치 ID를 해당 조건에 맞춰 바꾼다. [실제 파일럿에서 확인한 CSV](evidence/pilot_field_log.csv)

```bash
python3 trial_records.py add \
  --field-log ~/data/jint/field_log.csv --trial-id original_day_s1_01 \
  --phase main --environment original_day --layout S1 \
  --configuration ~/data/jint/field_config_01.json \
  --bag ~/data/bags/nav_original_day_s1_01 \
  --result ~/data/trial_outcomes/original_day_s1_01.json \
  --observed-outcome success --goal-check '현장 관찰자가 표시된 목표 구역 도달 확인'
```

위 예시는 실제로 제한시간 내 목표 구역에 도달했고 접촉·수동 개입이 없었던 회차에 사용한다. `--observed-outcome`은 파일 등록과 함께 현장 판정을 기록하며, `--goal-check`는 관찰자나 외부 영상의 판정 근거를 받는다. 실패는 `collision`, `timeout`, `nav_abort`, `manual_intervention` 중 해당 값을 사용한다. 복합 상황에서 확인한 사실은 `--contact true` 또는 `--intervention true`로 함께 기록한다. 무효 시도에는 `--observed-outcome invalid --invalid-reason '사유'`를 사용한다. 관찰 확인을 나중에 하려면 판정 옵션을 생략해 등록하고 CSV에서 해당 행을 채운다. Nav2 상태는 별도 열에 저장되며, 산출되지 않은 파일의 인수는 생략할 수 있다.

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
