# 📝 V-LiDAR 논문 작업 및 연구실 ICCAS/IJCAS/IEEE 아카이브
(V-LiDAR Paper Writing Workspace & CGV Lab Paper Archive)

본 디렉터리는 **단안 비전 기반 2D 가상 라이다(V-LiDAR)를 활용한 실내 자율주행 AMR 장애물 회피 연구**의 논문 작성 작업 공간이자, CGV 연구실에서 진행 중인 **SCIE Q2 저널(IEEE Access) 투고 및 ICCAS/IJCAS 연구 성과**를 집대성한 통합 아카이브입니다.

---

## 📂 폴더 구조 (Directory Structure)

```
paper/
├── README.md                                  # [본 문서] 논문 작업 및 아카이브 종합 마스터 가이드
├── submission/ (제출처/)                      # 11월 말 Q2 저널 투고처 심층 분석 및 타임라인 전략
│   └── README.md                              # IEEE Access 최우선 선정, Final Accept vs First Decision 비교
├── IEEE_Access/ ➔ docs/papers/IEEE_Access/    # [최우선 투고 대상] IEEE Access 12페이지 완성 원고 (심볼릭 링크)
├── v_lidar_manuscript_overleaf.zip            # Overleaf 즉시 업로드용 전체 압축본 (모듈형 초안)
│
├── manuscript/                                # 신규 논문 작성용 LaTeX 워크스페이스 (모듈형 제네릭 2단 초안)
│   ├── Makefile                               # 컴파일 및 Overleaf 패키징 자동화 스크립트
│   ├── main.tex                               # Master LaTeX 본문 (IEEE/IJCAS 2단 포맷)
│   ├── references.bib                         # 참고문헌 BibTeX 데이터베이스
│   ├── sections/                              # 챕터별 모듈화 LaTeX 소스
│   │   ├── 00_abstract.tex                    # 초록 (Abstract & Keywords)
│   │   ├── 01_introduction.tex                # 서론 (Introduction & Research Contributions)
│   │   ├── 02_related_work.tex                # 관련 연구 (Related Works & Lab Lineage)
│   │   ├── 03_methodology.tex                 # 제안 기법 (YOLOv11-seg, LUTs, Scan 변환)
│   │   ├── 04_experiments.tex                 # 실험 평가 (20회 반복 회피 검증 데이터)
│   │   ├── 05_discussion.tex                  # 고찰 및 한계 (사각지대 유도, 협소 복도 분석)
│   │   └── 06_conclusion.tex                  # 결론 및 향후 계획 (Conclusion & Future Work)
│   └── figures/                               # 논문 삽입용 고해상도 다이어그램 및 시각화 자료
│       ├── fig1_v_lidar_pipeline.png          # 전체 V-LiDAR 시스템 파이프라인
│       ├── fig2_nav2_costmap.png              # Nav2 코스트맵 마킹 및 레이트레이싱
│       ├── fig3_sector_distribution.png       # 141채널 부채꼴 섹터 각도 분포도
│       ├── fig4_yolo_benchmark.png            # 모델별 추론 속도 및 FPS 벤치마크
│       └── fig_framework_compact.png          # 논문 인쇄용 소형 아키텍처 다이어그램
│
└── submitted_papers/                          # 학회/저널 기제출 논문 및 피드백 아카이브
    ├── iccas/                                 # ICCAS 제출 논문 원본 및 리뷰
    │   ├── Vision-Based_2D_Scan_Generation_for_Obstacle_Avoidance_Using_Floor_Segmentation_revised.pdf
    │   ├── ICCAS_V-liadr_original.pdf
    │   ├── ICCAS_Reviewer_Comments.txt        # ICCAS 부편집장(AE) 심사 피드백 전문
    │   └── ICCAS_Submission_Metadata.md       # 논문 메타데이터 및 대응 분석 보고서
    ├── ijcas_lab_references/                  # CGV 연구실의 IJCAS 저널 게재 논문
    │   ├── README.md                          # IJCAS 게재 논문 3편 상세 분석 및 연구 계보
    │   └── ijcas_lab_papers.bib               # IJCAS 및 ICROS 논문 공식 BibTeX 데이터
    └── experimental_data/                     # 논문 보강용 정량 실험 데이터
        ├── Obstacle_Avoidance_Experiment_Results.md  # 20회 반복 실험 결과 및 사각지대 유도
        └── Obstacle_Avoidance_Experiment_Plan.md     # 실험 설계 및 시나리오 계획서
```

---

## 🎯 1. 11월 말 SCIE Q2 저널 투고처: IEEE Access

사용자 지침에 따라 **MDPI 저널을 전면 배제**하고, 11월 말까지 결과 확보(Final Accept 또는 First Decision)가 가능한 Q2 저널을 분석한 결과, **`IEEE Access`**가 최우선 투고처로 확정되었습니다.

* **저널 등급**: **IEEE / SCIE Q2 (2025 JIF 4.2)**
* **상세 분석 보고서**: [`paper/submission/README.md`](submission/README.md) (또는 [`paper/제출처/`](제출처/))
* **준비된 원고 위치**: [`docs/papers/IEEE_Access/`](../docs/papers/IEEE_Access/) (편의 링크: [`paper/IEEE_Access/`](IEEE_Access/))
* **Overleaf 즉시 투고 패키지**: [`docs/papers/IEEE_Access_Paper_Overleaf.zip`](../docs/papers/IEEE_Access_Paper_Overleaf.zip) (23.7 MB)
* **심사 주기**: 평균 4~6주(중앙값 30~35일), Binary Peer Review(단판 승부)로 11월 말 Final Accept 달성이 가능한 유일한 대안.
* **출판비(APC)**: 교신저자 황성수 교수님 IEEE Senior Member 20% 할인 ($1,728) 적용, 글로컬대학30 ANCHOR 연구비(`2026-ANCHOR-15-119`) 집행.

---

## 🏛️ 2. 연구실 ICCAS 및 IJCAS 연구 성과 아카이브

### A. ICCAS 제출 논문 (V-LiDAR 관련 논문)
* **논문명**: *Vision-Based 2D Scan Generation for Obstacle Avoidance Using Floor Segmentation*
* **저자**: 이현서, 유건민, 구현우, 황성수* (교신저자)
* **학회**: **ICCAS** (International Conference on Control, Automation and Systems)
* **형태**: 2-Page Extended Abstract
* **심사 피드백 (Associate Editor)**:
  * 장점: 카메라 기반 가상 2D 라이다 생성 아이디어의 실용성 및 경량성, 정직한 한계 수치 보고.
  * 지적/보완 필요: 단일 복도/단일 박스에 한정된 검증, 22% 실패율, 라이다 베이스라인 비교 부재.
* **대응 현황**:
  * 20회 정밀 반복 실험을 통해 성공률을 **90.0% (18/20회 성공, 충돌 0회)**로 향상.
  * 카메라 높이($1.05\text{m}$)와 틸트($2.0^\circ$)에 따른 최단 가시 거리($D_{\text{min}} = 2.178\text{m}$) 수학적 공식 유도 완료.

### B. CGV 연구실 IJCAS 저널 게재 논문 (연구 계보)
제어로봇시스템학회(ICROS)와 Springer가 공동 발행하는 **IJCAS (International Journal of Control, Automation and Systems)**에 게재된 연구실 대표 논문 3편이 [`submitted_papers/ijcas_lab_references/`](submitted_papers/ijcas_lab_references)에 정리되어 있습니다:

1. **Self-Calibration (2020)**:
   * *Fast and Accurate Self-calibration Using Vanishing Point Detection in Manmade Environments*
   * 저자: Sang Jun Lee, Sung Soo Hwang*
   * DOI: [10.1007/s12555-019-0284-1](https://doi.org/10.1007/s12555-019-0284-1)
2. **Depth Estimation for SLAM (2020)**:
   * *Real-time Depth Estimation Using Recurrent CNN with Sparse Depth Cues for SLAM System*
   * 저자: Sang Jun Lee, Heeyoul Choi, Sung Soo Hwang*
   * DOI: [10.1007/s12555-019-0350-8](https://doi.org/10.1007/s12555-019-0350-8)
3. **Visual Place Recognition (2019)**:
   * *Bag of Sampled Words: A Sampling-based Strategy for Fast and Accurate Visual Place Recognition in Changing Environments*
   * 저자: Sang Jun Lee, Sung Soo Hwang*
   * DOI: [10.1007/s12555-018-0790-6](https://doi.org/10.1007/s12555-018-0790-6)

---

## ✍️ 3. 논문 작성 및 컴파일 가이드

### A. IEEE Access 원고로 작업할 경우 (11월 말 목표 최우선 경로)
1. [`docs/papers/IEEE_Access_Paper_Overleaf.zip`](../docs/papers/IEEE_Access_Paper_Overleaf.zip)을 다운로드하여 Overleaf에 업로드합니다.
2. 모든 폰트, 스타일 파일(`ieeeaccess.cls`), 저자 사진, 고해상도 그림이 포함되어 있어 즉시 컴파일 가능합니다.

### B. 모듈화된 초안 워크스페이스에서 작업할 경우
1. 본 디렉터리의 [`v_lidar_manuscript_overleaf.zip`](v_lidar_manuscript_overleaf.zip)을 다운로드하여 Overleaf에 업로드합니다.
2. 로컬 리눅스 환경에서 컴파일할 경우:
   ```bash
   cd /home/cgv/ros2_ws/paper/manuscript
   make all
   ```
