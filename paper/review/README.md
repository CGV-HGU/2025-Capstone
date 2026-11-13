# IEEE Access 심사 결과 리뷰 및 후속 연구·재투고 플랜 (`Access-2026-51740`)

> **대상 논문**: *V-LiDAR: Real-Time Monocular Floor Segmentation and Virtual 2D Scan Synthesis for Autonomous Mobile Robot Obstacle Avoidance*  
> **투고 저널 / ID**: IEEE Access / `Access-2026-51740`  
> **판정 일자 / 결과**: `2026-11-12` / **Reject (Do not accept resubmissions)**  
> **담당 에디터(AE)**: Dr. Kumaradevan Punithakumar (`punithak@ualberta.ca`)  
> **기준 Git 브랜치**: `post`

---

## 1. 폴더 문서 구성

| 파일명 | 내용 요약 | 활용 목적 |
| :--- | :--- | :--- |
| **[`01_IEEE_Access_Decision_and_Raw_Reviews.md`](./01_IEEE_Access_Decision_and_Raw_Reviews.md)** | AE 결정문 및 Reviewer 1·2·3 리뷰·질의항목 원문 전문 | 리뷰어 정확한 워딩 대조 및 교수님 공유 |
| **[`02_Review_Analysis_and_Asset_Audit_Report.md`](./02_Review_Analysis_and_Asset_Audit_Report.md)** | 3인 리뷰어 교차 분석 + 저장소/가중치(`best.pt`)/설정(`omo_r1.yaml`) 실사 보고서 | 기존 보유 자산과 반드시 추가해야 할 실험 구분 |
| **[`03_Revision_and_Experiment_Action_Plan.md`](./03_Revision_and_Experiment_Action_Plan.md)** | 로봇 PC(`cgv`) 점검 명령어, Track A(무실험)/Track B(소규모 실차) 플랜, 차기 저널 전략 | 로봇 접속 후 점검 및 실험·개정 실행 |
| **[`04_Robot_PC_Asset_Audit_and_Log_Inventory.md`](./04_Robot_PC_Asset_Audit_and_Log_Inventory.md)** | 로봇 PC(`cgv`) 실물 자산 감사(40개 bag·Trash·모델·캐시) + Git 푸시 가능 로그 인벤토리 | 실제 PC 로그 기준 리뷰 대응 범위 확정 및 로컬 동기화 |
| **[`05_Advisor_Report_Summary.md`](./05_Advisor_Report_Summary.md)** | 교수님 보고용 핵심 요약(3인 리뷰 요약 + 개선 가능 범위 A/B) | 교수님 대면/메일 보고 및 보완 범위 공유 |

> **푸시 가능 추출 로그**: [`.gitignore`](../../.gitignore)를 정비하여 [`experiments/logs/`](../../experiments/logs/) 내 기존 68 KB 벤치마크 로그와 신규 추출 경량 로그([`experiments/logs/pc_extracted/`](../../experiments/logs/pc_extracted/README.md), 총 ~1.08 MiB)가 Git 추적을 통해 로컬 PC로 동기화될 수 있도록 정리했습니다.

---

## 2. 한눈에 보는 심사 판정 요약

| 리뷰어 | 체크리스트 평가 | 핵심 논조 및 요구사항 |
| :--- | :--- | :--- |
| **Reviewer 1** | **4개 항목 전부 `Yes`** (부적절 문헌 `No`) | 논문 구성과 기여는 인정. 단, **카메라 높이/틸트 변경 실험**, **경량 MDE(FastDepth/Lite-Mono) 비교**, **반사바닥·유리문·저조도·보행자 시나리오** 추가 요구. (Comment #3에 주제와 무관한 논문 3편 인용 제안 포함) |
| **Reviewer 2** | **가장 회의적** (전제 공격) | **$2.18\text{ m}$ 사각지대**와 초음파/ToF 보완 권고가 '카메라 단독 LiDAR 대체' 주장을 약화시킨다고 지적. 소프트웨어 TCO, 저사양 임베디드 보드 검증, 불확실성 정량화(OOD/calibration), 동적 장애물 지연 분석 부재 비판. |
| **Reviewer 3** | **가장 건설적** (재투고 청사진) | 실용적 문제·구현·솔직한 실패 분석을 호평. 단, **① 동일 로봇 물리 2D LiDAR 비교 부재, ② YOLO 데이터셋·학습·정확도 누락, ③ 2.5 m 단일 거리/단일 환경 한계, ④ `run08/12` 수정 후 재주행 미검증, ⑤ 과장된 포지셔닝 수정**을 5대 필수 과제로 제시. |

### 교수님 보고용 3줄 요약
> 교수님, 금일 IEEE Access (`Access-2026-51740`) 심사 결과 **리젝(재투고 불가)** 판정을 받았습니다. 총 3명의 리뷰어 중 1명은 평가 항목 전반에 긍정적(`Yes`), 1명은 2.18 m 사각지대 및 LiDAR 대비 효용성에 근본적 회의론을 제기했고, 나머지 1명은 실용적 문제의식과 솔직한 실패 분석을 호평하면서도 명확한 보완 항목을 짚어주었습니다. 에디터는 논문 서술이나 구성이 아닌 **① 동일 로봇 물리 LiDAR 비교 부재, ② YOLO 학습 데이터·성능 수치 누락, ③ Nav2 파라미터 수정 후 재주행 미검증, ④ 단일 홀·단일 거리 실험 한계** 등 **신규 실험이 필요한 사안이 많아** 단기 수정으로는 수용이 어렵다고 판단한 것으로 보이며, 향후 타 저널 투고 시에는 '완전한 LiDAR 대체' 대신 '개방형 실내(2.2–6.0 m) ROS 2 통합 계측 시스템'으로 스코프를 재정비하고 위 항목들을 보완하는 방향을 논의드리고 싶습니다.

---

## 3. 로봇 PC 접속 시 즉시 점검할 Quick Commands (`cgv@100.96.194.210`)

나중에 로봇 온보드 PC에 SSH로 접속하신 뒤 아래 순서대로 실행하여 **남아 있는 기존 데이터(`~/data/bags/`, 학습 데이터셋)와 물리 LiDAR 장치 연결 가능 여부**를 먼저 확인하세요.

```bash
# 1. 저장소 최신 리뷰 문서 동기화
cd ~/ros2_ws
git fetch origin
git checkout post
git pull origin post

# 2. 기존 주행 Rosbag 및 실험 로그 잔존 여부 확인
ls -lh ~/data/bags/
ls -lh ~/ros2_ws/experiments/logs/

# 3. YOLO 학습 원본 데이터셋/YAML 위치 확인 (R3 #2 보완용: 장수, split 확인)
find ~ -maxdepth 4 \( -name "data.yaml" -o -name "results.csv" -o -iname "*floor*seg*" \) 2>/dev/null

# 4. 물리 YDLIDAR / RPLIDAR 하드웨어 및 드라이버 설치 여부 확인 (R3 #1 보완용)
ls -l /dev/ttyLiDAR /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
ros2 pkg list | grep -E "ydlidar|rplidar"
```

상세한 오프라인/실차 실험 절차와 차기 저널 이식 가이드는 [`03_Revision_and_Experiment_Action_Plan.md`](./03_Revision_and_Experiment_Action_Plan.md)를 참고하세요.
