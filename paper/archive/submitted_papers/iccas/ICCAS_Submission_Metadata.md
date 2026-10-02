# 📄 ICCAS 제출 논문 메타데이터 및 심사 피드백 분석

---

## 1. 논문 기본 정보 (Paper Metadata)

| 항목 | 내용 |
| :--- | :--- |
| **논문 제목** | **Vision-Based 2D Scan Generation for Obstacle Avoidance Using Floor Segmentation** |
| **제출 학술대회** | **ICCAS** (International Conference on Control, Automation and Systems) |
| **저자진** | 이현서 (Hyunseo Lee)¹, 유건민 (Gunmin Yoo)², 구현우 (Hyunwoo Gu)², 황성수 (Sung Soo Hwang)³* (교신저자) |
| **소속 기관** | 한동대학교 (Handong Global University) AI·전산전자공학부 |
| **논문 형태** | Extended Abstract (2-Page 초록 논문) |
| **연구 지원** | • 글로컬대학 30 경북 ANCHOR 사업단 지원 (2026-ANCHOR-15-119)<br>• 한동대학교 학술연구비 (No. 202500590001) 및 과학기술정보통신부 한국연구재단(NRF) 지원 (No. RS-2025-24683458) |
| **제출 파일** | • [Vision-Based_2D_Scan_Generation_for_Obstacle_Avoidance_Using_Floor_Segmentation_revised.pdf](file:///home/cgv/ros2_ws/paper/submitted_papers/iccas/Vision-Based_2D_Scan_Generation_for_Obstacle_Avoidance_Using_Floor_Segmentation_revised.pdf) (최신 제출본)<br>• [ICCAS_V-liadr_original.pdf](file:///home/cgv/ros2_ws/paper/submitted_papers/iccas/ICCAS_V-liadr_original.pdf) (초기 제출본) |

---

## 2. 심사위원 / Associate Editor 심사 피드백 요약

> 원본 파일: [ICCAS_Reviewer_Comments.txt](file:///home/cgv/ros2_ws/paper/submitted_papers/iccas/ICCAS_Reviewer_Comments.txt)

### 긍정적 평가 (Strengths)
1. **아이디어의 실용성 및 경량성**: 고가의 물리적 라이다 센서 없이 단안 RGB 카메라와 바닥 세그멘테이션만을 이용해 Nav2 호환 가상 2D LaserScan을 생성하는 독창적이고 실용적인 접근법.
2. **명확하고 솔직한 한계 보고**: 최대 오차($\le 0.261\,\text{m}$), 10m 회피 성공률 78%(39/50회) 등 한계와 수치를 투명하게 명시함.

### 지적 사항 및 확장 필요점 (Critique & Revision Needs)
1. **실험 환경의 한계**: 단일 복도에서 단일 박스 장애물만을 대상으로 검증되어 실험적 일반화가 부족함.
2. **실패율(22%) 및 환경 민감도**: 광택 바닥(glossy floor)의 빛 반사 및 조명 변화에 따른 세그멘테이션 불안정성.
3. **베이스라인 비교 부재**: 물리적 라이다(Physical LiDAR) 또는 대체 비전 기반 기법과의 정량적 비교 부재.

---

## 3. 후속 연구 및 저널(IJCAS) 확장 대응 전략

1. **20회 정밀 반복 실험 수행 ([Obstacle_Avoidance_Experiment_Results.md](file:///home/cgv/ros2_ws/paper/submitted_papers/experimental_data/Obstacle_Avoidance_Experiment_Results.md))**:
   - 성공률을 기존 78%에서 **90.0% (18/20회 성공)**로 크게 끌어올림.
   - 평균 회피 시작 거리 $2.22 \pm 0.08\,\text{m}$, 평균 횡방향 회피 폭 $1.22 \pm 0.45\,\text{m}$ 도출.
2. **기하학적 사각지대 공식 유도**:
   - 카메라 설치 높이($H=1.05\,\text{m}$)와 틸트($\theta=2.0^\circ$), 수직 화각($\alpha_v=47.48^\circ$)에 따른 최소 지면 가시 거리 $D_{\text{min}} = \frac{1.05}{\tan(25.74^\circ)} \approx 2.178\,\text{m}$의 수학적 유도 완료.
3. **협소 복도 회피 불능 원인 규명**:
   - 회피 폭($1.22\,\text{m}$)과 코스트맵 팽창 반경($1.30\,\text{m}$)의 중첩으로 인한 코스트맵 트래핑(Costmap Trapping) 메커니즘을 규명하여, 넓은 개방 공간 실험의 타당성을 입증.
