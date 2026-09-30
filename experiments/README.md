# 🧪 자율주행 및 장애물 인식 2대 실험 가이드 (IEEE Access)

본 디렉터리는 **IEEE Access 논문**에 수록되는 2대 핵심 실험인 **[Track 1: 제안 V-LiDAR 시스템 실증]**과 **[Track 2: 단안 뎁스 Baseline(MiDaS / Depth Anything V2) 비교 평가]**를 수행하고 결과를 체계적으로 관리하기 위한 통합 도구 모음입니다.

> 📄 **전체 상세 프로토콜 및 실측 데이터는 [EXPERIMENT_PROTOCOL_AND_RESULTS.md](EXPERIMENT_PROTOCOL_AND_RESULTS.md)를 참고하세요.**

---

## 📂 1. 디렉터리 구성
```text
experiments/
├── EXPERIMENT_PROTOCOL_AND_RESULTS.md  # 📋 [필독] 2대 실험 마스터 프로토콜 및 결과 기록서
├── benchmark_depth_baselines.py        # 🔬 V-LiDAR vs MiDaS vs Depth Anything V2 벤치마크 (정적/Rosbag 동적)
├── run_experiment.py                  # 🚗 실시간 자율주행 통합 로깅 도구 (5Hz)
├── evaluate_freespace.py              # 🔍 Freespace Detection 정밀 벤치마크 및 오프라인 평가
├── configs/                           # 실험 프리셋 설정 파일
└── logs/                              # 📁 실험 결과 자동 저장 폴더
    ├── dynamic_trajectory_benchmark.csv # Rosbag 동적 프레임별 거리 추적 시계열 데이터
    ├── baseline_benchmark_report.md     # 모델별 Latency/FPS/MAE 마크다운 보고서
    └── baseline_latex_table.tex         # IEEE Access 논문 삽입용 완성된 LaTeX 표
```

---

## 🚗 2. 자율주행 실험 로거 (`run_experiment.py`)

로봇 자율주행을 시작할 때 백그라운드나 별도 터미널에서 실행해 두면, 주행 궤적과 V-LiDAR 장애물 감지 거리를 실시간 기록하고 종료 시 **시험 요약 리포트(`summary_report.md`)**를 자동 생성합니다.

### 실행 방법
```bash
# 기본 실행 (기본 맵 NTH4F, 기본 시나리오명)
python3 experiments/run_experiment.py

# 시나리오 및 맵 명시 실행
python3 experiments/run_experiment.py --scenario box_avoidance_run1 --map NTH4F
```

### 기록되는 핵심 데이터
1. **주행 기동성**: 주행 시간(s), 총 이동 거리(m), 평균 및 최대 선속도(m/s)
2. **Visual SLAM 추적 안정성**: 카메라 위치추정(Tracking) 유지율 (%)
3. **장애물 안전성**: 주행 중 장애물 최근접 거리(m), 50cm 이내 근접 경보 횟수, 최종 통과(PASS) 여부

---

## 🔍 3. Freespace Detection 성능 평가기 (`evaluate_freespace.py`)

로봇을 주행시키지 않고도, 다양한 조명/바닥 상태에서 **YOLO 바닥 세그멘테이션과 V-LiDAR 변환 품질을 정량 평가**할 수 있습니다.

### 실행 방법
```bash
# 10초간 실시간 평가 수행
python3 experiments/evaluate_freespace.py --duration 10 --scenario glossy_floor_test

# 카메라 ID 변경 및 평가 시간 지정
python3 experiments/evaluate_freespace.py --duration 30 --device-id 0 --scenario obstacle_1m
```

### 측정 지표
* **추론 지연 시간(Latency)**: Mean, P50, P95, P99 백분위수 및 실시간 FPS
* **바닥 인식 면적 비율**: 전체 화면 중 보행 가능 바닥으로 인식된 비율 (%)
* **분기별 거리 측정**: Left (-35°~-12°), Center (-11°~+11°), Right (+12°~+35°) 구역별 최소 장애물 거리
* **바닥 인식 안정성**: 프레임 간 표준편차를 통한 바닥 반사광/플리커링 감지

---

## 💡 다회차 실험 분석 팁
여러 번의 실험(`run1`, `run2`, ...)을 수행한 후 `experiments/logs/`에 생성된 각 폴더의 `summary_report.md`를 비교하면, 조명 변화나 장애물 위치에 따른 성공률과 접근 거리를 객관적인 지표로 정리할 수 있습니다.
