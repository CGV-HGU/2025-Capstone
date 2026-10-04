# 📋 V-LiDAR 논문 수정 권고사항 종합 리포트 (팀 공유 및 논의용)

> **대상 원고**: [`paper/workspace/IEEE_Access/main.tex`](../IEEE_Access/main.tex) (IEEE Access 투고용)  
> **현재 브랜치**: `post`  
> **작성 목적**: 팀원 간 수정 필요 사항 공유, 학술적 스코프 점검, 및 최종 제출 전 합의안 도출  
> **작성 일자**: 2026-10-04  
> **원고 상태**: **원본 소스(`main.tex`)는 일절 수정하지 않고 보존 중** (팀 협의 후 일괄 반영 예정)

---

## 📌 목차 및 상세 문서 안내

이 폴더(`paper/workspace/revision_recommendations/`)에는 영역별 상세 분석 보고서와 즉시 적용 가능한 LaTeX Diff 코드가 작성되어 있습니다:

| 문서명 | 주요 내용 |
| :--- | :--- |
| **[`01_AUTHOR_ORDER_AND_BIOGRAPHIES_RECOMMENDATION.md`](./01_AUTHOR_ORDER_AND_BIOGRAPHIES_RECOMMENDATION.md)** | 저자 순서(구현우 외 5인), 5인 공동 1저자(Equal Contribution) 각주, 14페이지 약력 레이아웃 안전 마진 실측 결과 |
| **[`02_FIGURE6_AND_FIGURE7_SCOPE_AUDIT.md`](./02_FIGURE6_AND_FIGURE7_SCOPE_AUDIT.md)** | Figure 6 'Dynamic' 명칭 모순 및 **Figure 7 내부 ORB-SLAM3(PangolinViewer) 노출 결함 해결 방안** |
| **[`03_WHOLE_PAPER_SCOPE_AND_REVIEWER_VULNERABILITY_AUDIT.md`](./03_WHOLE_PAPER_SCOPE_AND_REVIEWER_VULNERABILITY_AUDIT.md)** | Line 587 심사위원 피드백 유출 문구 삭제, 오도메트리/센서퓨전 표현 정합, 협로 주행 서술 톤 완화 등 |
| **[`04_EXACT_DROP_IN_DIFF_PROPOSALS.md`](./04_EXACT_DROP_IN_DIFF_PROPOSALS.md)** | 팀 합의 후 `main.tex`에 즉시 반영할 수 있는 **16개 완전 검증된 Drop-in Diff 블록 모음** |

---

## 🎯 핵심 수정 필요 사항 요약 (Executive Summary)

현재까지 파악된 수정 권고 사항은 크게 **4가지 카테고리**로 분류됩니다:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                       V-LiDAR 수정 권고사항 구조                             │
├──────────────────────┬──────────────────────┬───────────────────────────────┤
│ 1. 저자 및 표제부    │ 2. 시각자산(Fig 6, 7)│ 3. 스코프 및 심사위원 방어    │
│  - 6인 저자 순서 재배치│  - Fig 6 '정적' 정합화│  - Line 587 리뷰어 유출 삭제 │
│  - 5인 공동 1저자 표기│  - Fig 7 SLAM 뷰어 제거│ - 좁은 복도 서술 톤 개선     │
│  - 14페이지 약력 수용 │  - 복도 vs 홀 명칭 일치│  - 초음파/IMU 과장 표현 완화 │
└──────────────────────┴──────────────────────┴───────────────────────────────┘
```

---

## 1. 저자 정보 및 14페이지 약력 정비 (Author Ordering & Biographies)

### A. 저자 순서 재배치
팀 내 최종 기여도 및 역할 협의에 따라 아래 순서로 재배치가 제안되었습니다:
1. **구현우** (`HYUNWOO GU`) — **공동 제1저자 (Equal Contribution)**
2. **유건민** (`GUNMIN YOO`) — **공동 제1저자 (Equal Contribution)**
3. **이현서** (`HYUNSEO LEE`) — **공동 제1저자 (Equal Contribution)**
4. **강현모** (`HYUN-MO KANG`) — **공동 제1저자 (Equal Contribution)**
5. **이민석** (`MIN-SEOK LEE`) — **공동 제1저자 (Equal Contribution)**
6. **황성수 교수님** (`SUNG SOO HWANG`) — **교신저자 (Corresponding Author, Senior Member, IEEE)**

### B. 표제부 및 러닝 헤더 동기화
* **Title Footnote (각주)**:  
  `\textit{Hyunwoo Gu, Gunmin Yoo, Hyunseo Lee, Hyun-Mo Kang, and Min-Seok Lee contributed equally to this work.}`  
  (학생 5인 전원의 동등 기여 공식 명시)
* **Running Header**:  
  1저자가 구현우 학생으로 변경됨에 따라 `Lee \headeretal` $\to$ `Gu \headeretal`로 변경 필요.
* **이메일 순서**: 표기된 저자 순서에 맞춰 이메일 나열 순서 일치화 (`21800030@...`, `gunminy@...`, `hslee@...`, `hmkang012@...`, `glen@...`, `sshwang@...`).

### C. 14페이지 레이아웃 안전성 시뮬레이션 결과
* 변경된 순서(구현우 $\to$ 유건민 $\to$ 이현서 $\to$ 강현모 $\to$ 이민석 $\to$ 황성수)로 6인 약력과 사진($1.0 \times 1.25$ in)을 배치하여 컴파일 시뮬레이션을 수행한 결과:
  - **총 페이지 수: 정확히 14.0페이지 (15페이지 오버플로우 0행)**
  - Page 14 Column 1 하단 잔여 여백: **24.2 pt**
  - Page 14 Column 2 하단 잔여 여백: **104.5 pt** (`\EOD` 마크 포함)
  - 14페이지 규격을 100% 안전하게 충족함을 사전 확인했습니다.

---

## 2. 시각 자산(Figure 6, Figure 7) 스코프 적합성 검토

### A. Figure 6 (`experiment2_setup.jpeg`) — 정적 장애물 명칭 정합화
* **현상**: 사진 상 장애물은 정지해 있는 단일 골판지 상자($0.4 \times 0.4 \times 0.5$\,m)이나, 본문과 캡션(Lines 459, 465)에서 **"Dynamic obstacle avoidance"**라는 용어를 과도하게 사용함.
* **심사위원 지적 위험**: 로봇 공학 심사위원은 보행자 등 움직이는 객체에 대한 추적/동적 장애물 회피가 아니므로 '과장(Overclaiming)'으로 지적할 수 있음.
* **권고 조치**: 
  - 본문 및 캡션을 **"Closed-loop autonomous obstacle avoidance (폐루프 자율 장애물 회피)"**로 정제.
  - 실험 공간(폭 약 6m 홀)을 "corridor" 대신 **"open testing hall"**로 표기하여 Section VI-B(협로 한계 분석)와의 용어 충돌 방지.

### B. Figure 7 (`nav2_obstacle_avoidance_sequence.png`) — 🚨 긴급 조치 필요
* **현상**:
  - Figure 7의 3분할 시퀀스 패널 중앙($x \in [450, 1650]$\,px, **패널 폭의 36.2% 차지**)에 **`PangolinViewer` (ORB-SLAM3 3D 포인트클라우드 및 카메라 Frustum 맵)** 창이 버젓이 노출되어 있음!
* **심사위원 지적 위험**:
  - 논문은 시종일관 *"사전 지도나 SLAM 없이(mapless without SLAM or pre-built maps) 순수 가상 2D 스캔으로 주행한다"*고 주장하는데, 그림 한가운데에 SLAM 맵 뷰어가 있으면 *"실제로는 Visual SLAM으로 위치를 추정하고 맵을 만든 것이 아닌가?"*라는 심각한 모순 지적을 받을 수 있음.
* **권고 조치 (팀 선택 필요)**:
  - **옵션 1 (강력 권장 - 무손실 이미지 크롭)**: 중앙의 PangolinViewer 창($x \in [450, 1650]$)을 잘라내고, 좌측 세그멘테이션 영상과 우측 Nav2 RViz 화면만 나란히 결합하여 교체.
  - **옵션 2 (본문 및 캡션 해명 추가)**: PangolinViewer는 데이터 로깅용 백그라운드 프로세스일 뿐, 실제 Nav2 로컬 코스트맵과 회피 기동에는 일절 개입하지 않았음을 캡션에 명시.

---

## 3. 논문 전반의 스코프 및 심사위원 방어 서술 정비

### A. 🚨 Line 587 리비전 유출 문구 삭제 (필수)
* **현상**: Table 6 어블레이션 서술 도입부(Line 587)에 아래 문장이 그대로 남아있음:  
  *"The ablation trajectory in Table 6 clearly demonstrates how each engineering design directly resolves the **reviewer's critique**:"*
* **위험**: 학술대회(ICCAS) 심사위원 피드백에 답변하던 문구가 본문에 유출된 것으로, IEEE Access 투고 시 이전 거절/수정 이력을 노출하게 됨.
* **조치**: *"demonstrates the cumulative performance contribution of each post-processing and acceleration module:"*로 교체.

### B. 오도메트리 및 센서 퓨전 표현 정정 (Line 513)
* **현상**: *"Localization relied solely on wheel odometry fused with camera optical geometry."*로 적혀 있으나, 단안 카메라 기하학은 오도메트리에 EKF로 퓨전된 것이 아니라 로컬 코스트맵에 장애물 스캔(`/scan`)으로 들어간 것임.
* **조치**: 로봇 상태 추정은 휠 오도메트리에 의존하고 가상 스캔은 로컬 코스트맵에 공급되었음을 사실대로 기술.

### C. 좁은 복도 주행(Section VI-B) 서술 톤 개선 (Line 621)
* **현상**: 제목이 "Narrow Corridor Infeasibility (협로 주행 불가)"로 되어 있어 제안 기법의 가치를 깎아내리는(Self-detracting) 느낌을 줌.
* **조치**: **"Operational Boundaries in Confined Spaces and Geometric Scaling (제한된 공간에서의 운용 한계 및 기하학적 스케일링)"**으로 세련되게 재정의.

### D. 향후 연구 센서 융합(Section VI-C) 및 IMU 피치 보상(Section VI-D) 과장 완화
* **초음파 융합(Line 634)**: "guarantees zero-blind-spot coverage" $\to$ "represents a promising architectural roadmap"으로 완화 (실측 전이므로 '보장' 단언 회피).
* **IMU 피치 보상(Line 640)**: 20회 실험에서는 정적 2D LUT를 사용했음을 명시하고, 수식 (23)은 급가감속 로봇을 위한 해석적 수식(Analytical formulation)임을 명확히 한정.

### E. Table 5 처리 속도 표기 일치화 (Line 547)
* Table 5 내 Proposed FPS가 `77.4 FPS`로 적혀 있으나, 초록, 서론, Table 6에는 `78.4 FPS`로 표기되어 있으므로 `78.4 FPS`로 통일 권장.

---

## 4. 변경 불필요 확인 항목 (False Alarm 교정)

* **카메라 TF 마운트 좌표 ($x = -0.55$\,m) 부호 확인**:
  - 논문 수식 (Line 296)에 $\mathbf{T} = [-0.55, 0, 0]^T$로 음수로 표기되어 있어 오류 의혹이 제기되었음.
  - **실제 코드 추적 결과**: `omo_r1.urdf`(Lines 71–72)에 `origin xyz="-0.5 0 0.2"` (구동축 뒤 0.5m에 마운트 기둥 설치 후 전방 주시) 및 ROS 드라이버 노드(`fake_lidar_with_tf.py` Line 68)에 `-0.55`가 명시되어 있음.
  - **결론**: **$-0.55$\,m는 실제 로봇 하드웨어 세팅과 100% 일치하므로 수정하지 않음**.

---

## 5. 팀 내 결정 필요 사항 (Next Steps for Team)

1. **저자 순서 최종 승인**:
   - 구현우 $\to$ 유건민 $\to$ 이현서 $\to$ 강현모 $\to$ 이민석 $\to$ 황성수 교수님 순서 및 5인 공동 1저자 표기로 확정할 것인지 여부.
2. **Figure 7 이미지 처리 방향 결정**:
   - [옵션 1] 이미지 크롭으로 `PangolinViewer` 창을 완전히 삭제할 것인지,  
   - [옵션 2] 이미지는 유지하고 캡션에 비연동 백그라운드 뷰어임을 명시할 것인지.
3. **일괄 반영 진행 여부**:
   - 위 항목들에 대해 팀원 합의가 완료되면 [`04_EXACT_DROP_IN_DIFF_PROPOSALS.md`](./04_EXACT_DROP_IN_DIFF_PROPOSALS.md)에 준비된 16개 Diff 블록을 `main.tex`에 일괄 적용하고 최종 PDF 빌드 및 깃 푸시를 진행할 수 있습니다.
