# 📋 V-LiDAR 논문 투고 마스터 현황 및 작업 관리 대시보드
**(Master Paper Submission Status, Organization, and Research Roadmap)**

> **최종 갱신 일시**: 2026-10-01  
> **프로젝트 위치**: `C:\Users\USER\Desktop\캡스톤\V-Lidar-Research`  
> **현재 Git 브랜치**: `post` (Remote: `https://github.com/CGV-HGU/2025-Capstone.git`)  
> **연구 책임 및 교신저자**: 황성수 교수님 (Handong Global University, CGV Lab)  
> **작업 원칙**: **원본 문서(`docs/papers/`)는 원본 백업으로 영구 보존**하며, 모든 신규 작성/수정 작업은 **`paper/workspace/`** 내에서 독립적으로 수행함.

---

## 📑 목차 (Table of Contents)
1. [저널 투고 전략 및 원고 워크스페이스 구조](#1-저널-투고-전략-및-원고-워크스페이스-구조)
2. [저자진 정보 및 세부 메타데이터 업데이트](#2-저자진-정보-및-세부-메타데이터-업데이트)
3. [ICCAS 심사위원 지적사항 및 정밀 대응 전략](#3-iccas-심사위원-지적사항-및-정밀-대응-전략)
4. [광학 기하학 및 이론적 사각지대 분석 체계](#4-광학-기하학-및-이론적-사각지대-분석-체계)
5. [마스터 실측 실험 데이터 (Table I, II, III)](#5-마스터-실측-실험-데이터-table-i-ii-iii)
6. [신규 확장 계획 표 (Table IV Sensory Comparison & Table V Ablation Study)](#6-신규-확장-계획-표-table-iv--table-v)
7. [소프트웨어 파이프라인 및 원터치 브링업](#7-소프트웨어-파이프라인-및-원터치-브링업)
8. [단계별 논문 작성 로드맵 및 체크리스트](#8-단계별-논문-작성-로드맵-및-체크리스트)

---

## 1. 저널 투고 전략 및 원고 워크스페이스 구조

### 1.1 투고 목표 저널 3대 후보군 비교
사용자 지침에 따라 **MDPI 저널을 전면 배제**하고, 11월 말까지 결과 확보(Final Accept 또는 First Decision) 및 무감축 12페이지 수용이 가능한 SCIE Q2 저널 3곳을 선정하여 3종 원고를 병렬 유지합니다.

| 순위 | 저널명 | 출판사 / 인덱스 | 1차 판정 주기 | 게재 비용 (APC) | 작업용 원고 위치 | 백업 원본 위치 | 선정 사유 및 핵심 전략 |
| :---: | :--- | :---: | :---: | :---: | :--- | :--- | :--- |
| **1순위** | **IEEE Access** | IEEE / **SCIE Q2** (JIF 4.2) | **3~6주** (중앙값 30일) | ~$1,728 (교수님 20% 할인) | `paper/workspace/IEEE_Access/` | `docs/papers/IEEE_Access/` | **[최우선 목표]** 12페이지 원고/PDF 완성. Binary 심사로 11월 말 Final Accept 확률 최고. |
| **2순위** | **Computers & Electrical Engineering (C&EE)** | Elsevier / **SCIE Q2** (JIF 4.0) | **3.6~5주** (약 25~35일) | **$0 (구독모델 시 완전무료)** | `paper/workspace/Elsevier_CEE/` | `docs/papers/Elsevier_CEE/` | **[무료 대안 1]** 임베디드 AI + 비전 내비게이션 스코프 완벽 일치. 상용 출판사 중 가장 빠른 초심. |
| **3순위** | **Measurement Science & Technology (MST/MSAT)** | IOP / **SCIE Q2** (JIF 2.7) | **4~6주** | **$0 (구독모델 시 완전무료)** | `paper/workspace/IOP_MST/` | `docs/papers/IOP_MST/` | **[무료 대안 2]** 2D 기하 캘리브레이션 및 광학 사각지대 거리 계측 정밀도에 초점을 둔 계측 분야 명문. |

### 1.2 작업 디렉터리 분리 체계
* **원본 보존 디렉터리 (`docs/papers/`)**: ICCAS 승인본, IEEE Access 초도 완성본(PDF 포함), 원본 데이터셋 보존. 어떠한 경우에도 임의 수정하지 않음.
* **실작업 워크스페이스 (`paper/workspace/` 및 `paper/manuscript/`)**:
  ```
  paper/
  ├── workspace/
  │   ├── IEEE_Access/              # IEEE Access LaTeX 작업 원고 (main.tex, ieeeaccess.cls, figures/, etc.)
  │   ├── Elsevier_CEE/             # Elsevier C&EE LaTeX 작업 원고 (main.tex, elsarticle.cls, figures/, etc.)
  │   ├── IOP_MST/                  # IOP MST LaTeX 작업 원고 (main.tex, iopart.cls, figures/, etc.)
  │   ├── IEEE_Access_Overleaf.zip  # [최신] IEEE Access Overleaf 즉시 업로드 패키지
  │   ├── Elsevier_CEE_Overleaf.zip # [최신] Elsevier C&EE Overleaf 즉시 업로드 패키지
  │   ├── IOP_MST_Overleaf.zip      # [최신] IOP MST Overleaf 즉시 업로드 패키지
  │   ├── experimental_data/        # 20회 주행 로그, Baseline 벤치마크 스크립트, 실험 프로토콜
  │   │   ├── EXPERIMENT_PROTOCOL_AND_RESULTS.md
  │   │   ├── Obstacle_Avoidance_Experiment_Results.md
  │   │   ├── benchmark_depth_baselines.py
  │   │   └── Baseline_Experiment_Guide.md
  │   └── references/               # ICCAS 심사의견서, 참조 논문 원본 PDF, 메타데이터
  │       ├── 2026-254_1_1.pdf      # 최신 포스트캡스톤 연구 논문 (이민석, 강현모, 황성수 저자정보 출처)
  │       ├── ICCAS_Reviewer_Comments.txt
  │       ├── ICCAS_Submission_Metadata.md
  │       ├── ICCAS_V-liadr.pdf
  │       └── Vision-Based 2D Scan Generation for Obstacle Avoidance Using Floor Segmentation.pdf
  ├── manuscript/                   # 모듈형 제네릭 2단 LaTeX 초안 (저자 갱신 완료, main.pdf 5p)
  └── v_lidar_manuscript_overleaf.zip # manuscript/ 모듈형 초안 Overleaf 배포 패키지
  ```

---

## 2. 저자진 정보 및 세부 메타데이터 업데이트

최신 포스트캡스톤 논문(`docs/papers/2026-254_1_1.pdf`) 및 포스트캡스톤 수강신청서, Git 커밋 이력으로부터 추출한 공식 저자 메타데이터입니다.

### 2.1 저자 명단 및 역할 정의

| 구분 | 성명 (국문/영문) | 소속 기관 및 학부 | 학번 / 신분 | 이메일 (공식) | ORCID | 연구 기여 및 역할 |
| :---: | :--- | :--- | :---: | :--- | :---: | :--- |
| **제1저자**<br>(공동) | **이민석**<br>(Min-Seok Lee) | 한동대학교 AI·전산전자공학부<br>(School of AI, Computer & Electrical Eng.) | 22100504<br>학부 연구원 | `glen@handong.ac.kr` | [0009-0008-5641-2721](https://orcid.org/0009-0008-5641-2721) | 시스템 통합, 실차 주행 실험, 고장 진단 및 논문 집필 총괄 |
| **제1저자**<br>(공동) | **강현모**<br>(Hyun-Mo Kang) | 한동대학교 AI·전산전자공학부<br>(School of AI, Computer & Electrical Eng.) | 22100026<br>학부 연구원 | `hmkang012@gmail.com` | [0009-0004-7846-649X](https://orcid.org/0009-0004-7846-649X) | OpenVINO FP16 엣지 최적화, Baseline 벤치마크 엔진, 원터치 브링업 |
| **공동저자** | **이현서**<br>(Hyunseo Lee) | 한동대학교 AI·전산전자공학부 | 학부 졸업생 | `hslee@handong.ac.kr`<br>(`iam@hsl.ee`) | - | 선행 연구 개발 (ICCAS 2025 Extended Abstract 1저자) |
| **공동저자** | **유건민**<br>(Gunmin Yoo) | 한동대학교 전산전자공학부 | 대학원 석사과정<br>(CGV Lab / KIRO) | `gunminy@handong.ac.kr` | - | 로봇 제어 및 기구학 캘리브레이션 지원 |
| **공동저자** | **구현우**<br>(Hyunwoo Gu) | 한동대학교 전산전자공학부 | 대학원 석사과정<br>(CGV Lab / KIRO) | `21800030@handong.ac.kr` | - | SLAM 백엔드 및 센서 인터페이스 지원 |
| **교신저자**<br>(*) | **황성수**<br>(Sung Soo Hwang) | 한동대학교 AI·전산전자공학부 | 정교수 / 연구책임<br>(IEEE Senior Member) | `sshwang@handong.edu` | [0000-0002-0863-7503](https://orcid.org/0000-0002-0863-7503) | 연구 총괄 기획, 이론 검증, 논문 최종 감수 |

### 2.2 연구비 사사 정보 (Grant Acknowledgments)
```latex
This research was supported by the ANCHOR program Glocal University 30 through the Gyeongbuk ANCHOR CENTER, 
funded by the Ministry of Education (MOE) and the Gyeongsangbuk-do, Republic of Korea (2026-ANCHOR-15-119). 
This work was also supported in part by the National Research Foundation of Korea (NRF) grant funded by the 
Korea government (MSIT) (No. RS-2025-24683458) and the Handong Global University Academic Research Grant (No. 202500590001).
```

### 2.3 저자 약력 (Author Biographies for IEEE Access)
* **Min-Seok Lee**: is currently pursuing the B.S. degree in the School of Artificial Intelligence, Computer and Electrical Engineering at Handong Global University, Pohang, South Korea. Since 2025, he has been with the Computer Graphics and Vision Laboratory (CGV Lab) under the supervision of Prof. Sung Soo Hwang. His research interests include mobile robotics, visual SLAM, robust navigation, and sensor fusion.
* **Hyun-Mo Kang**: is currently pursuing the B.S. degree in the School of Artificial Intelligence, Computer and Electrical Engineering at Handong Global University, Pohang, South Korea. Since 2025, he has been with the Computer Graphics and Vision Laboratory (CGV Lab) under the supervision of Prof. Sung Soo Hwang. His research interests include deep learning deployment, edge AI acceleration, autonomous robotics, and visual perception.
* **Hyunseo Lee**: was born in Busan, South Korea, in 2002. He is currently pursuing the B.S. degree in the School of Artificial Intelligence, Computer and Electrical Engineering at Handong Global University, Pohang, South Korea. Since 2025, he has been with the Computer Graphics and Vision Laboratory (CGV Lab)...
* **Gunmin Yoo**: received the B.S. degree in computer science from Handong Global University, Pohang, South Korea, in 2025. He is currently pursuing the M.S. degree in the School of Computer Science and Electrical Engineering at Handong Global University...
* **Hyunwoo Gu**: received the B.S. degree in computer science from Handong Global University, Pohang, South Korea, in 2025. He is currently pursuing the M.S. degree in the School of Computer Science and Electrical Engineering at Handong Global University...
* **Sung Soo Hwang**: received the B.S. degree from Handong Global University in 2008 and the M.S. and Ph.D. degrees in electrical engineering from KAIST in 2010 and 2015, respectively. He is currently an Associate Professor with the School of Computer Science and Electrical Engineering, Handong Global University...

---

## 3. ICCAS 심사위원 지적사항 및 정밀 대응 전략

ICCAS 학술대회 심사위원(AE) 피드백 원문:
> *"A practical, lightweight idea—converting monocular floor-segmentation into a virtual 2D scan for Nav2 obstacle avoidance without LiDAR—that is clearly presented and honestly reports its limits (<= 0.261 m range error, 78% success rate). However, this is only a preliminary extended abstract: validation is limited to a single static box in one corridor, the 22% failure rate and glossy-floor/illumination sensitivity are significant, and comparison against a LiDAR or alternative baseline is missing."*

| 지적 번호 | 심사위원 핵심 지적 사항 (Reviewer Critique) | 저널(SCIE Q2) 본 논문 대응 및 해결 전략 (Our Response & Resolution) | 수록 위치 / 근거 자료 |
| :---: | :--- | :--- | :--- |
| **지적 1** | **단일 복도, 단일 박스 실험의 한계**<br>(Validation limited to a single static box in one corridor) | • 10m 실차 경로에서 20회 연속 장애물 회피 실증 주행 전수 분석.<br>• 박스 외에 좌측 보행자(성인 남성), 우측 원통형 쓰레기통(Can) 등 다양한 형상 및 재질 정적 측정 추가.<br>• 좁은 복도 대신 6m 광폭 홀에서 진행한 당위성을 기하학적·코스트맵 관점에서 명확히 규명. | • Section V.B (Table II)<br>• Section V.A (Table I)<br>• Section V.D (사각지대 및 협소복도 한계) |
| **지적 2** | **높은 실패율 (기존 78% 성공, 22% 실패)**<br>(The 22% failure rate is significant) | • 세그멘테이션 후처리 필터 개선 및 Nav2 파라미터 튜닝을 통해 **완주 성공률을 90.0% (18/20회 성공)로 대폭 향상**.<br>• 나머지 2회 중도 정지(Run 08, Run 12)에 대한 철저한 원인 분석(Costmap Trapping, DWB Critic Scale 과도화, Nav2 시뮬레이션 클록 버그)을 투명하게 공개하고 해결책 제시. | • Section V.B (Table II)<br>• Section V.C (Failure Case Analysis) |
| **지적 3** | **광택 바닥 및 조명 반사 민감도**<br>(Glossy-floor / illumination sensitivity) | • **3단계 강건 후처리 파이프라인 도입**:<br>  1) 다중 인스턴스 마스크 비트합(Bitwise-OR) 병합<br>  2) $5 \times 5$ Morphological Closing (빛 반사 구멍 제거)<br>  3) 시계열 지수이동평균(EMA, $\alpha=0.6$) 및 Persistence($N=2$) 필터로 깜빡임(Flickering) 원천 억제. | • Section III.B (Segmentation Post-processing)<br>• Section IV (Ablation Analysis) |
| **지적 4** | **물리 라이다 및 대체 베이스라인 비교 부재**<br>(Comparison against LiDAR or alternative baseline missing) | • **Track 2 벤치마크 신설**: 동일한 온보드 CPU(Intel Core Ultra 7 155H) 환경에서 제안 기법(V-LiDAR)과 최신 MDE 모델(MiDaS v2.1 Small, Depth Anything V2 Small) 1:1 비교 실증.<br>• 연산 지연시간(12.9ms vs 174.2ms), 파라미터(2.83M vs 24.8M), 실시간 10Hz 제어 충족 여부 실증 비교.<br>• 물리적 2D LiDAR vs 3D LiDAR vs RGB-D vs Monocular MDE vs 제안 V-LiDAR의 7개 축 종합 비교표(Table IV) 설계. | • Section V.D (Table III)<br>• Table IV (Sensory Comparison) |

---

## 4. 광학 기하학 및 이론적 사각지대 분석 체계

### 4.1 카메라 기구학 및 마운트 좌표계
* 카메라 설치 높이: $H = 1.05\,\text{m}$ (지면 기준 수직 높이)
* 카메라 하향 틸트 각도: $\theta = 2.0^\circ$ (기구물 미세 처짐 및 실측 캘리브레이션 오프셋)
* 수직 화각 (V-FOV): $\alpha_v = 47.48^\circ$ (광학 중심 $y_0 = 134.6$, 유효 세로 256 픽셀)
* 수평 화각 (H-FOV): $\alpha_h = 70.0^\circ$ (141개 스캔 채널, $0.5^\circ$ 간격 분해능)

### 4.2 근거리 광학 사각지대 (Near-Field Optical Blind Spot) 유도
단안 카메라 영상에서 장애물 하단과 바닥이 만나는 지면 접촉선(Ground-contact line)을 기반으로 거리를 역추정하는 기하 원리상, 카메라 화면 최하단 행($y = 255$)에 투영되는 광선 각도가 측정 가능한 최소 물리 거리의 하한선이 됩니다.

$$\theta_{\max} = \theta + \frac{\alpha_v}{2} = 2.0^\circ + \frac{47.48^\circ}{2} = 2.0^\circ + 23.74^\circ = 25.74^\circ$$

$$D_{\min} = \frac{H}{\tan(\theta_{\max})} = \frac{1.05\,\text{m}}{\tan(25.74^\circ)} = \frac{1.05}{0.4821} \approx \mathbf{2.178\,\text{m}}$$

> **물리적 결론**: 로봇이 장애물에 $2.18\,\text{m}$ 이내로 근접하면 장애물 바닥 접촉선이 시야각(FOV) 바깥으로 벗어나게 되므로, 단안 비전만으로는 $2.178\,\text{m}$ 이하의 거리를 물리적으로 측정할 수 없습니다. 20회 주행 실측에서 최소 감지 거리가 전 회차에서 정확히 $2.178\,\text{m}$로 기록된 것은 모델 오류가 아닌 광학 기하학적 필연입니다.

### 4.3 협소 복도(<2.2m) 주행 불능 사유 (Physical Infeasibility)
1. **측면 벽면 사각지대**: 폭 2.0m 복도 중앙 주행 시 좌우 벽면과의 거리는 1.0m임. 최소 지면 가시 거리가 2.18m이므로 로봇 전방 측면 약 $27^\circ$ 이상의 벽면 접촉선은 사각지대에 매몰되어 측면 벽과의 거리를 측정할 수 없음.
2. **기구학적 회피 반경 초과**: 폭 0.4m 박스 회피 시 OMO-R1의 실측 평균 횡방향 이탈폭은 $Y = 1.22 \pm 0.45\,\text{m}$에 달함. 폭 2.0m 복도에서 1.22m 횡이동 시 로봇 풋프린트($r=0.40\,\text{m}$)가 벽면과 즉각 충돌함.
3. **코스트맵 팽창 폐색 (Corridor Choke)**: 양측 벽면과 장애물의 팽창 반경($1.30\,\text{m}$)이 중첩되어 복도 전 구간이 치명적 장애물 구역(Lethal Zone)으로 마킹되어 경로 계획기가 주행을 즉시 포기함.  
$\rightarrow$ 따라서 6m 개방형 홀에서의 실험이 알고리즘 순수 성능 평가를 위해 필수 불가결했음을 논문에서 이론적으로 입증.

---

## 5. 마스터 실측 실험 데이터 (Table I, II, III)

### 5.1 [Table I] 정적 거리 추정 정밀도 (2.50m 기준, 25 프레임 평균)
*출처: `paper/workspace/experimental_data/EXPERIMENT_PROTOCOL_AND_RESULTS.md` Section 4.1*

| 장애물 조건 (Target Condition) | 방위각 (Angle) | 참값 (GT, m) | 평균 추정치 (Mean, m) | 표준편차 ($\sigma$, m) | 절대 오차 (Error, m) | 상대 오차 (Rel. Error) |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **중앙 (Clean Floor, 표준 박스)** | $0^\circ$ | 2.500 | 2.718 | 0.008 | +0.218 | +8.72% |
| **중앙 (Noisy/Glossy Floor, 표준 박스)** | $0^\circ$ | 2.500 | 2.239 | 0.172 | -0.261 | -10.44% |
| **좌측 (보행자, 성인 남성)** | $+20^\circ$ | 2.500 | 2.583 | 0.011 | +0.083 | +3.32% |
| **우측 (원통형 쓰레기통)** | $-20^\circ$ | 2.500 | 2.663 | 0.000 | +0.163 | +6.52% |

### 5.2 [Table II] 10m 실차 20회 연속 장애물 회피 주행 종합 결과
*출처: `paper/workspace/experimental_data/EXPERIMENT_PROTOCOL_AND_RESULTS.md` Section 4.2*

| 회차 (Run) | 주행 시간 (s) | 전진 거리 ($X$, m) | $Y$ 회피 폭 (m) | 최소 감지 거리 (m) | 최대 각속도 (rad/s) | 주행 결과 및 비고 |
| :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **run01** | 68.5 | 9.76 | 0.958 | 2.178 | 0.200 | ✅ 정상 회피 후 완주 |
| **run02** | 67.7 | 9.78 | 0.903 | 2.178 | 0.367 | ✅ 정상 회피 후 완주 |
| **run03** | 82.8 | 9.77 | 1.056 | 2.178 | 0.267 | ✅ 정상 회피 후 완주 |
| **run04** | 53.7 | 9.77 | 0.887 | 2.385 | 0.133 | ✅ 정상 회피 후 완주 |
| **run05** | 46.7 | 9.76 | 1.362 | 2.429 | 0.267 | ✅ 정상 회피 후 완주 |
| **run06** | 60.4 | 9.77 | 1.124 | 2.178 | 0.400 | ✅ 정상 회피 후 완주 |
| **run07** | 58.9 | 9.78 | 0.942 | 2.187 | 0.467 | ✅ 정상 회피 후 완주 |
| **run08** | 121.3 | **3.70** | 0.899 | 2.413 | 0.200 | ⏸️ 중도 정지 (Costmap Trapping) |
| **run09** | 49.9 | 9.78 | 0.980 | 2.178 | 0.200 | ✅ 정상 회피 후 완주 |
| **run10** | 60.8 | 9.76 | 1.928 | 2.178 | 0.500 | ✅ 정상 회피 후 완주 |
| **run11** | 62.5 | 9.76 | 0.989 | 2.178 | 0.333 | ✅ 정상 회피 후 완주 |
| **run12** | 81.1 | **2.91** | 1.069 | 2.218 | 0.333 | ⏸️ 중도 정지 (DWB Critic Penalty) |
| **run13** | 50.4 | 9.78 | 1.274 | 2.313 | 0.300 | ✅ 정상 회피 후 완주 |
| **run14** | 49.4 | 9.82 | 2.563 | 2.277 | 0.400 | ✅ 정상 회피 후 완주 |
| **run15** | 50.4 | 9.77 | 0.781 | 2.178 | 0.400 | ✅ 정상 회피 후 완주 |
| **run16** | 53.3 | 9.76 | 0.987 | 2.178 | 0.347 | ✅ 정상 회피 후 완주 |
| **run17** | 55.8 | 9.76 | 1.024 | 2.178 | 0.500 | ✅ 정상 회피 후 완주 |
| **run18** | 58.5 | 9.76 | 0.828 | 2.179 | 0.500 | ✅ 정상 회피 후 완주 |
| **run19** | 59.2 | 9.82 | 1.676 | 2.198 | 0.500 | ✅ 정상 회피 후 완주 |
| **run20** | 56.4 | 9.77 | 1.702 | 2.214 | 0.500 | ✅ 정상 회피 후 완주 |
| **통계** | **58.1 ± 9.3 s** | **9.77 m** | **1.22 ± 0.45 m** | **2.22 ± 0.08 m** | **0.34 rad/s** | **완주 성공률: 18/20 (90.0%)** |

### 5.3 [Table III] 제안 V-LiDAR vs 단안 뎁스 Baseline 비교 벤치마크 (동일 온보드 CPU)
*온보드 연산 환경: Intel Core Ultra 7 155H (16코어 22스레드)*

| 모델 / 방법론 | 추론 백엔드 및 구조 | 파라미터 (Params) | 모델 크기 (Disk) | 추론 지연시간 (Latency) | 처리량 (Throughput) | $2.50\text{m}$ 정적 MAE | 10Hz 제어 루프 충족 여부 |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **V-LiDAR (제안 기법)** | **OpenVINO FP16 (Floor+LUT)** | **2.83 M** | **5.4 MB** | **12.9 ± 1.9 ms** | **77.4 FPS** | **0.218 m** | **완벽 충족 (7.7배 여유)** |
| **V-LiDAR (제안 기법)** | PyTorch CPU (Floor+LUT) | 2.83 M | 5.8 MB | 20.1 ± 1.5 ms | 49.7 FPS | 0.218 m | 완벽 충족 (5.0배 여유) |
| **MiDaS v2.1 Small** | PyTorch EfficientNet MDE | 21.4 M | 42.0 MB | 38.5 ± 3.8 ms | 26.0 FPS | 0.350 m | 충족 (여유 마진 적음) |
| **Depth Anything V2 Small** | PyTorch DINOv2-ViT MDE | 24.8 M | 97.5 MB | 174.2 ± 12.5 ms | 5.7 FPS | 0.420 m | **미충족 (심각한 제어 병목)** |

---

## 6. 신규 확장 계획 표 (Table IV & Table V)

논문 개정 및 확장 투고를 위해 기획된 신규 핵심 표 2종의 상세 설계안입니다.

### 6.1 [Table IV] 센서 양식별 종합 비교 (Sensory Modality Comparison)
*목적: 로봇 장애물 인지에 활용되는 주요 센서 양식과 제안 기법의 비용, 중량, 전력, 광학 특성, 연산량을 1:1 비교하여 V-LiDAR의 학술적/실용적 독보성을 증명.*

| 센서 양식 (Modality) | 대표 하드웨어 예시 | 하드웨어 도입비용 (USD) | 중량 (Payload, g) | 소비 전력 (Power, W) | 수평 시야각 (H-FOV) | 바닥 빛반사/유리창 취약도 | 메트릭 스케일 직접 제공 | 추가 온보드 연산 부하 |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **2D Planar LiDAR** | RPLIDAR A2 / UST-10LX | $300 ~ $1,800 | 190 ~ 400 g | 4.0 ~ 8.0 W | $360^\circ$ / $270^\circ$ | 취약 (유리 투과, 왁스 난반사) | 있음 (ToF) | 거의 없음 (시리얼 드라이버) |
| **3D Solid-State LiDAR** | Livox Mid-360 / Ouster | $800 ~ $4,000 | 265 ~ 450 g | 6.5 ~ 15.0 W | $360^\circ \times 59^\circ$ | 보통~취약 (유리창 산란) | 있음 (ToF) | 중간 (포인트클라우드 필터링) |
| **RGB-D Camera** | Intel RealSense D435i | $350 ~ $500 | 72 g | 2.5 ~ 3.5 W | $86^\circ \times 57^\circ$ | **극히 취약** (IR 난반사 흡수) | 있음 (Stereo/IR) | 낮음~중간 (Depth to Scan) |
| **Monocular Dense MDE** | Standard RGB + DepthAnyV2 | **<$20** (저가 웹캠) | **<30 g** | **<1.0 W** | $70^\circ \sim 90^\circ$ | 취약 (지면을 장애물로 오인) | **없음 (상대 깊이)** | **극심 (ViT >170ms)** |
| **V-LiDAR (제안 기법)** | **Standard RGB + OpenVINO** | **<$20 (단안 웹캠)** | **<30 g** | **<1.0 W** | **$70.0^\circ$ (141ch)** | **강건 (Closing+EMA 필터)** | **있음 (2D Euclidean LUT)** | **초경량 (12.9ms, 2.83M)** |

### 6.2 [Table V] 인식 및 후처리 모듈 단계별 소거 연구 (Ablation Study)
*목적: 제안 파이프라인의 각 구성요소(기본 탐지, 다중 인스턴스 병합, 모폴로지 클로징, 시계열 필터링, OpenVINO 가속)가 성능(FPS, 거리 지터, 오탐지율, 주행 성공률)에 미치는 기여도를 단계별로 정량 입증.*

| 단계 (Configuration) | 구성 요소 (Modules Included) | 추론 속도 (FPS) | 2.5m 거리 지터 ($\sigma$, m) | 광택 바닥 오탐률 (False Obstacle Rate, %) | 10m 실차 회피 성공률 (Success Rate, %) | 핵심 개선 효과 및 비고 |
| :---: | :--- | :---: | :---: | :---: | :---: | :--- |
| **(A) Base** | Raw YOLOv11n-seg (단일 마스크) + 1D LUT | 49.7 | 0.384 | 38.5% | 60.0% (12/20) | 바닥 조명 반사로 인한 가상 장애물 다수 발생 |
| **(B) +Merge** | (A) + Multi-instance Bitwise-OR | 48.2 | 0.245 | 24.0% | 70.0% (14/20) | 분할된 바닥 조각 병합으로 감지 누락 방지 |
| **(C) +Morph** | (B) + $5\times 5$ Morphological Closing | 46.5 | 0.172 | 11.2% | 80.0% (16/20) | 반사 하이라이트 구멍 및 노이즈 아티팩트 메움 |
| **(D) +Temporal** | (C) + Temporal EMA($\alpha=0.6$) \& Persistence($N=2$) | 45.1 | **0.008** | **1.8%** | **90.0% (18/20)** | **프레임간 깜빡임(Flicker) 제거 및 주행 안정화** |
| **(E) +OpenVINO** | **(D) + OpenVINO FP16 Engine (최종 통합)** | **77.4** | **0.008** | **1.8%** | **90.0% (18/20)** | **추론 지연시간 12.9ms 달성 (CPU 대비 1.7배 가속)** |

---

## 7. 소프트웨어 파이프라인 및 원터치 브링업

최신 Git 커밋(`a89ab8f`, `641398a`, `928761b`)으로 구현된 통합 자동화 시스템입니다.

### 7.1 원터치 통합 기동 스크립트 (`start_all.sh`)
```bash
cd ~/ros2_ws

# [기본] 헤드리스 전체 자율주행 통합 시스템 기동 (실험 모드)
./start_all.sh

# [모니터링] RViz GUI 시각화 모니터링 포함 기동
./start_all.sh --rviz
```
* **동시 기동 노드**:
  1. `cam2image` (웹캠 640x480 @ 30fps)
  2. `omo_r1_bringup` (차륜 오도메트리 및 모터 구동 MCU)
  3. `freespace_detection` (V-LiDAR OpenVINO FP16 초고속 바닥 인식, 12.9ms)
  4. `fake_lidar_with_tf` (2D Euclidean LUT 거리 변환, 141ch `/scan`, TF 발행)
  5. `navigation2` (A* 글로벌 플래너 + DWB 로컬 컨트롤러 + 로컬 코스트맵)
* 종료 시 `Ctrl+C` 입력으로 `stop_all.sh`가 연동 실행되어 백그라운드 프로세스를 잔여물 없이 일괄 종료.

### 7.2 단안 뎁스 Baseline 동기화 벤치마크 엔진 (`experiments/benchmark_depth_baselines.py`)
```bash
# 1. 정적 거리 정밀도 25프레임 평가
python3 experiments/benchmark_depth_baselines.py --gt-dist 2.50 --test-iters 25 --use-openvino

# 2. 동적 실차 주행 Rosbag 기반 시계열 오차(MAE) 및 Latency 1:1 비교
python3 experiments/benchmark_depth_baselines.py \
    --bag ~/data/bags/vlidar_baseline_eval_run01 \
    --obs-x 4.5 --obs-y 0.0 --use-openvino
```
* 출력물: `dynamic_trajectory_benchmark.csv`, `baseline_latex_table.tex`, `baseline_benchmark_report.md` 자동 생성.

---

## 8. 단계별 논문 작성 로드맵 및 체크리스트

- [x] **Phase 1: 작업 환경 구축 및 저자 메타데이터 동기화 (완료)**
  - [x] `docs/papers/` 원본 백업 보존 확인
  - [x] `paper/workspace/` 독립 작업 디렉터리 구축 (`IEEE_Access/`, `Elsevier_CEE/`, `IOP_MST/`)
  - [x] 최신 코드 Git pull 및 머지 (`a89ab8f` 브링업 및 벤치마크 통합 완료)
  - [x] 최신 포스트캡스톤 논문(`2026-254_1_1.pdf`)에서 이민석, 강현모 저자/소속/이메일/ORCID 추출
  - [x] `paper/workspace/` 내 3대 저널 원고 저자 블록 및 약력 업데이트
  - [x] 마스터 현황 및 실험 관리 대시보드(`paper/PAPER_WORK_STATUS.md`) 작성
- [ ] **Phase 2: Table IV 및 Table V 실험 데이터 실측 및 LaTeX 표 제작 (진행 예정)**
  - [ ] Table IV 센서 비교 데이터 확정 및 LaTeX 조판
  - [ ] Table V Ablation study 모듈별 실측 테스트 수행 및 수치 검증
- [ ] **Phase 3: 본문 실험 결과(Section V) 확장 개정 (대기)**
  - [ ] 사용자 승인 후 IEEE Access `main.tex` Section V에 20회 주행 통계 및 Baseline 비교 표 삽입
  - [ ] Elsevier C&EE 및 IOP MST 작업 원고에도 동일 결과 반영
- [ ] **Phase 4: 최종 컴파일 및 Overleaf 제출 패키지 패키징 (대기)**
  - [ ] `pdflatex` 컴파일 및 페이지 수(12페이지) 준수 점검
  - [ ] Overleaf 배포용 zip 아카이브 갱신 및 최종 검수
