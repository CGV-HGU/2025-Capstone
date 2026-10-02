# 🔬 단안 뎁스 Baseline (MiDaS / Depth Anything V2) 비교 실험 가이드

본 문서는 **IEEE Access 논문 보강**을 위해 제안 기법(V-LiDAR: `YOLOv11n-seg` + LUT)과 대표적인 단안 뎁스 추정(MDE) Baseline인 **`MiDaS Small`** 및 **`Depth Anything V2 Small`**의 정량 비교 실험을 수행하는 전체 절차를 안내합니다.

---

## 📌 1. 실험 목표 및 비교 매트릭스

동일한 로봇 온보드 CPU(Intel Core Ultra 7 155H) 환경에서 다음 4대 핵심 지표를 측정합니다:

1. **모델 파라미터 수 (`Params, M`)**: 모델의 체급 및 메모리 점유도
2. **추론 지연 시간 (`Latency, ms`)**: 1프레임당 순수 처리 시간 (Mean ± Std, P95)
3. **센서 갱신 주기 (`Throughput, FPS / Hz`)**: 실시간 제어 루프 충족 여부
4. **$2.50\text{m}$ 정적 거리 추정 오차 (`MAE, m`)**: 실제 줄자 실측값 대비 오차

---

## ⚙️ 2. 환경 설정 및 의존성 패키지 설치

로봇 온보드 PC 또는 테스트 PC에서 필요한 패키지를 설치합니다:

```bash
# Hugging Face transformers 및 timm (Depth Anything V2 & MiDaS 로드용)
pip install transformers timm
```

---

## 🚀 3. [실험 1] 오프라인 벤치마크 (속도, 파라미터, 정적 오차 측정)

로봇을 주행시키지 않고 온보드 PC에서 즉시 3개 모델의 성능을 비교 측정합니다.

```bash
# 기본 실행 (30회 반복 측정, 2.50m 기준)
python3 experiments/benchmark_depth_baselines.py

# 복도 실제 촬영 이미지 또는 특정 사진으로 측정할 경우:
python3 experiments/benchmark_depth_baselines.py --image path/to/corridor_box.jpg --gt-dist 2.50

# OpenVINO 가속 활성화 (V-LiDAR)
python3 experiments/benchmark_depth_baselines.py --use-openvino
```

### 📋 자동 생성되는 결과물:
1. **터미널 콘솔 요약표**: 모델별 파라미터 수, Latency, FPS, MAE 출력
2. **`experiments/logs/baseline_benchmark_report.md`**: 마크다운 형태의 상세 분석 보고서
3. **`experiments/logs/baseline_latex_table.tex`**: **IEEE Access 논문에 복사&붙여넣기할 수 있는 완성된 LaTeX 표 소스**

---

## 🤖 4. [실험 2] 로봇 실차 Nav2 10m 장애물 회피 비교 주행 (선택)

실제 로봇에서 Baseline 모델(Depth Anything V2)로 `/scan`을 발행하여 Nav2 회피 기동을 비교 검증할 때 사용합니다.

### Step 1: Baseline 뎁스 노드 구동
`floor_detector` 및 `fake_lidar_with_tf` 대신 Baseline 노드를 실행합니다:

```bash
# 터미널 1: 카메라 구동
ros2 run image_tools cam2image --ros-args -p device_id:=0 -p width:=640 -p height:=480

# 터미널 2: Depth Anything V2 2D 라이다 변환 노드 실행
python3 scripts/baseline_depth_scan_node.py --ros-args -p model_type:=depth_anything

# (또는 MiDaS 실행 시)
python3 scripts/baseline_depth_scan_node.py --ros-args -p model_type:=midas
```

### Step 2: Nav2 자율주행 및 10m 상대 목표 전송
```bash
# 터미널 3: 로봇 MCU 및 Nav2 구동
ros2 launch omo_r1_bringup omo_r1_mcu.launch.py
ros2 launch omo_r1_navigation2 navigation2.launch.py use_sim_time:=false

# 터미널 4: 전방 10m 회피 명령 전송
python3 scripts/test_avoidance_goal.py -d 10.0
```

---

## 📝 5. 논문(IEEE Access) 반영 방법

1. `experiments/logs/baseline_latex_table.tex`에 생성된 표 코드를 [`docs/papers/IEEE_Access/main.tex`](../papers/IEEE_Access/main.tex)의 **Section V-A (Experiment 1)** 뒤에 삽입합니다.
2. 본문에 단안 뎁스 모델 대비 제안 기법의 실시간성(12.8ms vs 174ms) 및 지면 기하학 기반 거리 정밀도의 우수성을 1~2문단 추가합니다.
3. `main.tex`를 컴파일하여 최종 PDF를 생성합니다:
   ```bash
   cd docs/papers/IEEE_Access
   pdflatex main.tex && bibtex main && pdflatex main.tex && pdflatex main.tex
   ```
