# IEEE Access 심사 분석 및 저장소 보유 자산 실사 보고서 (`Access-2026-51740`)

- **작성 일자**: `2026-11-13`
- **검토 대상 원고**: `paper/workspace/IEEE_Access/main.tex` (커밋 `e2be012`)
- **검토 목적**: (1) 3인 리뷰어 지적사항 교차 구조화, (2) 현재 코드/가중치/로그 기준 "바로 쓸 수 있는 것 vs 추가 실험이 반드시 필요한 것" 판별

---

## 1. 심사 결과 해석: 왜 Major Revision 대신 '재투고 불가(Reject)'였는가?

IEEE Access는 리뷰어들이 **단순 텍스트/수식 수정이 아니라 새로운 물리 실험 및 베이스라인 추가**를 요구할 경우, 다회차 리비전 대신 `Reject (Do not accept resubmissions)` 결정을 내리는 경향이 강합니다.

### 리뷰어 3인 성향 및 평가 구조
1. **Reviewer 1 (표면적 긍정 + 대규모 실험 요구 + 비정상적 인용 제안)**
   - IEEE Access 4대 체크리스트(기여도·기술적 건전성·서술 완성도·참고문헌 적절성) 모두 **`Yes`**, 부적절 문헌 **`No`** 표기.
   - 그러나 본문에서 (i) 카메라 높이/틸트 변화 실험, (ii) 경량 MDE(`FastDepth`, `Lite-Mono`) 비교, (iii) 반사바닥·유리문·저조도·동적 보행자 테스트를 요구함.
   - *특이사항*: Comment #3에 본 연구와 무관한 논문 3편(*PatchTST 전력부하 예측, YOLO-TCS 교통표지판, 손 열흔적 추정*)을 DOI 없이 삽입하여 Q4 시스템 윤리 공지를 우회한 정황이 있음.
2. **Reviewer 2 (근본적 회의론 — 사실상 Strong Reject 요인)**
   - **"결국 $2.18\text{ m}$ 사각지대 때문에 초음파/ToF를 추가할 것이라면, 저가 2D LiDAR를 쓰는 편이 훨씬 단순하고 신뢰할 수 있지 않은가?"**라며 논문의 핵심 모티베이션(카메라 단독 LiDAR 대체)을 정면 비판.
   - 하드웨어 BOM만 강조하고 딥러닝 데이터 구축·학습·캘리브레이션 비용(TCO)을 누락한 점, 고사양 Intel Core Ultra 7 155H CPU 벤치마크만으로 저가 임베디드 보드 적용성을 입증하지 못한 점, 불확실성(confidence calibration / OOD) 및 동적 장애물 지연 분석 부재를 지적.
3. **Reviewer 3 (가장 공정하고 건설적 — 재투고 합격을 위한 5대 청사진)**
   - *"문제가 실용적이고 구현이 가볍지 않으며, 실패 분석이 다른 시스템 논문보다 솔직하다"*고 긍정 평가.
   - 동시에 본 논문의 실질적 기여를 **"새로운 인식 원리"가 아닌 "ROS 2 Nav2 통합 시스템 엔지니어링(System Integration)"**으로 규정하고, 합격을 위해 반드시 채워야 할 **5가지 구체적 결함**을 명시함.

> **총평**: 투고 직전 반영한 교수님 피드백(Table 6 SIL Replay 명시, MDE affine 스케일 보정, Claim 톤다운, 최신 레퍼런스 10편, AI 활용 공시)은 **논문 구성·가독성(R2 Q3 호평) 및 문헌 적절성(전원 부적절 문헌 `No`) 방어에 주효**했습니다. 반면 일정상 유보했던 실차 추가 실험과 [ICCAS 심사평](../../docs/reviews/comment.txt)에서 이미 경고된 구조적 약점(단일 환경·단일 박스·LiDAR 비교 부재)이 그대로 결정적 탈락 사유로 작용했습니다.

---

## 2. 리뷰어 3인 교차 분석 (공통 지적 vs 핵심 개별 결함)

### A. 3명 전원(3/3)이 공통으로 지적한 핵심 문제
| 공통 쟁점 | R1 지적 | R2 지적 | R3 지적 | 원고 내 현재 상태 (`paper/workspace/IEEE_Access/main.tex`) |
| :--- | :---: | :---: | :---: | :--- |
| **1. 실험 공간·장애물 다양성 부족** | #4 | #8, #10 | #3 | 정적 평가는 `2.50 m` 1지점(25프레임), 주행 평가는 약 `6 m` 폭 단일 홀·정지 박스 1개만 존재. 유리문·저조도·이질 바닥·보행자 주행 없음. |
| **2. 비교 Baseline 타당성 부족** | #2 | #4, Q4 | #1 | Table 5에서 느린 ViT MDE(Depth Anything V2)와 MiDaS만 비교. **동일 로봇 물리 2D LiDAR 비교(0건)** 및 **경량 실시간 MDE/Virtual LiDAR 비교(0건)**. |
| **3. 포지셔닝 과장 vs $2.18\text{ m}$ 한계** | #1 | #1, #9 | #5, Q1 | Abstract/Intro/Conclusion이 광범위한 LiDAR 대안처럼 서술되나, 실제로는 $D_{\min}=2.18\text{ m}$ 제약 및 초음파/ToF 추가 권고(Sec. VI-C)와 충돌. $(h, \gamma)$ 변경 실측도 미비. |

### B. Reviewer 3가 정확히 잡아낸 재현성·검증 결함 2가지
1. **세그멘테이션 모델 학습/검증 프로토콜 전면 누락 (`R3 #2`)**
   - Section III-B(Line 174)에 `"fine-tuned on custom indoor facility datasets under varying illumination"` 한 줄만 기재되어 있고, **데이터셋 출처·규모·라벨링 기준·train/val/test 분할·증강·학습 하이퍼파라미터·세그멘테이션 정확도(mIoU/mAP)**가 전혀 보고되지 않음.
2. **`run08`·`run12` 원인 수정 후 재주행 검증 부재 (`R3 #4`)**
   - Section VI-A(Lines 612–620)에서 `r_inflation=0.55 m`, `BaseObstacle.scale=2.5`, `use_sim_time: False`로 고치면 *"ensures uninterrupted path re-alignment"*라고 단정했으나, 수정 설정으로 재실행한 주행 결과표를 제시하지 않음.

### C. Reviewer 2의 시스템 심층 지적 4가지
- **`R2 #2` 소프트웨어 복잡도(TCO) 누락**: Table 1이 하드웨어 BOM(`<$20`)·무게·전력만 비교하고 데이터 수집·어노테이션·재학습·캘리브레이션 오버헤드를 생략함.
- **`R2 #5` Temporal Filter 동적 지연 미분석**: 추론 시간(`12.9 ms`)만 강조하고, EMA($\alpha=0.6$)와 Persistence($N_{\mathrm{onset}}=2$ @ `10 Hz` $\rightarrow$ 최소 `200 ms` 인지 지연)가 이동 장애물 회피에 미치는 시간/거리 지연을 분석하지 않음.
- **`R2 #6` 고사양 온보드 PC 한계**: Intel Core Ultra 7 155H (`32 GB RAM`) 결과를 기반으로 저가 경량 로봇 적용성을 일반화했으나, 저사양 임베디드 보드(Jetson Orin Nano, Raspberry Pi 등)에서의 검증이 없음.
- **`R2 #7` 불확실성 정량화(Calibration / OOD) 부재**: 고정 `c_conf = 0.15` 기반 점추정(point estimate)만 출력하며, 저신뢰 예측이나 OOD(미학습 바닥) 환경의 안전 Fallback 분석이 없음.

---

## 3. 저장소(코드·가중치·로그) 정밀 실사 결과: 지금 무엇이 있고 무엇이 없는가?

로봇 PC 접속 전 현재 브랜치(`post`) 내 실제 파일을 전수 검증한 결과입니다.

### [그룹 1] 추가 실험 없이 저장소 내 이미 존재하는 자산 (`바로 활용 가능`)
1. **YOLOv11n-seg 학습 설정 및 검증 정확도 (`R3 #2` 부분 해결)**
   - [`src/freespace_detection/scripts/best.pt`](../../src/freespace_detection/scripts/best.pt) 체크포인트 메타데이터(`train_args`, `train_metrics`) 및 [`metadata.yaml`](../../src/freespace_detection/scripts/best_openvino_model/metadata.yaml)에서 확인된 실제 수치:
     - **사전학습/입력/클래스**: `yolo11n-seg.pt`, 입력 `320×256` (`imgsz=320`), 2클래스(`0: person`, `1: floor`)
     - **학습 하이퍼파라미터**: `epochs=100`, `batch=16`, `optimizer=auto`, `lr0=0.01`, `lrf=0.01`, `momentum=0.937`, `weight_decay=0.0005`, `warmup_epochs=3.0`
     - **데이터 증강(Augmentation)**: `mosaic=1.0` (`close_mosaic=10`), `fliplr=0.5`, `translate=0.1`, `scale=0.5`, `hsv_h=0.015`, `hsv_s=0.7`, `hsv_v=0.4`, `auto_augment=randaugment`, `erasing=0.4`
     - **검증(Validation) 성능 지표**:
       - Mask: Precision **`0.805`** (`0.80468`), Recall **`0.761`** (`0.76136`), **mAP50 `0.802`** (`0.80222`), **mAP50-95 `0.562`** (`0.56231`)
       - Box: Precision `0.776`, Recall `0.780`, mAP50 `0.804`, mAP50-95 `0.593`
   - *부족한 정보*: 원본 데이터셋의 **출처, 총 이미지 수, train/val/test 분할 개수, 라벨링 기준**은 로컬 PC 저장소에 없으므로 로봇 PC 또는 학습 서버에서 확인 필요.

2. **기존 주행 및 정적 측정 데이터 (`Table 1~6`)**
   - `experiments/EXPERIMENT_PROTOCOL_AND_RESULTS.md` 및 `docs/papers/Obstacle_Avoidance_Experiment_Results.md`의 20회 주행표·2.50 m 정적 측정 데이터는 **이미 `IEEE_Access/main.tex`에 100% 반영 완료됨**. 논문에 미수록된 여분의 최신 실차 로그는 저장소 내에 없음.

3. **수식·서술 기반 즉시 보완 항목 (`R2 #2·#5`, `R3 #5`)**
   - EMA/Persistence 동적 지연 수식 정량화, TCO 비교 논의 추가, '개방형 실내 2.2–6.0 m ROS 2 통합 계측 시스템'으로의 포지셔닝 전면 재조정은 텍스트/수식 수정만으로 즉시 가능.

### [그룹 2] 로봇 PC(`~/data/bags/`)의 기존 Rosbag으로 오프라인 실행 가능한 자산
- 현재 Windows 로컬 저장소에는 `.db3` / `.mcap` 원본 bag 및 CSV 결과 파일이 존재하지 않으며(`experiments/logs/.gitkeep`만 존재), 스크립트(`scripts/record_rosbag.sh`, `experiments/run_dual_bag_benchmark.py`)상 모든 주행 bag은 **로봇 PC(`cgv@100.96.194.210`)의 `~/data/bags/`**에 저장됩니다.
- 로봇 PC에 `vlidar_run01`, `vlidar_run02` 및 20회 bag이 남아 있다면 **로봇 주행 없이** 오프라인으로 다음을 추가할 수 있음:
  - 경량 MDE(`FastDepth`, `Lite-Mono`) 추론 속도/MAE/오탐률 벤치마크 추가 (`R1 #2`)
  - 기존 bag 프레임 대상 confidence 임계값(`c_conf`) 민감도 및 오차 분포 분석 (`R2 #7`)

### [그룹 3] 저장소에 아예 없어 반드시 추가 실차·센서 실험이 필요한 4가지
1. **동일 로봇 물리 2D LiDAR 동시 비교 (`R3 #1`)**: `src/omo_r1_bringup/param/omo_r1_lidar.yaml`(YDLIDAR 설정)은 있으나, V-LiDAR와 동시에 `/scan`을 녹화·비교한 로그가 0건.
2. **수정된 Nav2 파라미터 실차 재주행 (`R3 #4`)**: 코드상 `src/omo_r1_navigation2/param/omo_r1.yaml`에는 `use_sim_time: False`, `BaseObstacle.scale: 5.0`, `inflation_radius: 0.75`로 중간 절충 수정이 반영되어 있으나, 이를 적용해 다시 주행한 Table 4 후속 검증 로그가 없음.
3. **다중 거리(2.2–6.0 m)·유리/저조도/이질 바닥·보행자 시나리오 (`R1 #4`, `R2 #8·#10`, `R3 #3`)**: 2.50 m 단일 거리 외 구간별 정밀도 및 도전 시나리오 데이터 전무.
4. **카메라 높이($h$)/틸트($\gamma$) 변경 실측 (`R1 #1`)**: 수식 유도만 있고 각도 변경에 따른 $D_{\min}$ 변화 실측 데이터 없음.

---

## 4. 기존 대안 저널 워크스페이스 동기화 주의사항

저장소 내 기존에 생성된 대안 저널 초안 폴더들은 **2026년 10월 1일 시점**에서 멈춰 있습니다:
- `paper/workspace/IOP_MST/main.tex` (수정일 `2026-10-01`)
- `docs/papers/Elsevier_CEE/main.tex` / `paper/workspace/Elsevier_CEE/`

이후 `paper/workspace/IEEE_Access/main.tex`에 반영된 핵심 개선사항(**Table 6 SIL Offline Replay 투명화, MDE affine scale 보정 수식, 과도한 Claim 톤다운, 최신 문헌 10편, 저자 순서 확정 및 AI 공시**)이 대안 저널 원고에는 반영되어 있지 않으므로, 차기 투고 시 **반드시 최신 `IEEE_Access/main.tex`를 원천 베이스(Single Source of Truth)로 삼아 이식**해야 합니다.
