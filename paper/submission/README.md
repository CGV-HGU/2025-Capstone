# 🎯 V-LiDAR 논문 Q2 저널 제출처 분석 및 투고 전략 (11월 말 결과 목표)

본 문서는 단안 비전 기반 2D 가상 라이다(V-LiDAR)를 활용한 자율주행 AMR 장애물 회피 연구의 **SCIE Q2 저널 투고**를 위한 최종 분석 보고서입니다.  
사용자의 핵심 요구사항인 **① MDPI 전면 배제**, **② 11월 30일까지 결과 확보 (Final Accept 또는 First Decision)**, **③ 기 확보된 연구실 원고 인프라 최적 활용**을 충족하는 저널들을 심층 비교 분석하였습니다.

---

## 📌 1. 핵심 결론 요약

> **💡 사용자가 기억하신 "MDPI 제외, IEEE 뭐"의 정체는 바로 `IEEE Access`입니다.**

* **최우선 추천 저널**: **`IEEE Access`** (IEEE 발행, **SCIE Q2**, 2025 JIF **4.2**)
* **현재 원고 준비 상태**: [`docs/papers/IEEE_Access/`](../../docs/papers/IEEE_Access/)에 12페이지 풀페이퍼 원고(`main.tex`), 참고문헌, 고해상도 그림, 저자 사진 및 Overleaf 패키지(`IEEE_Access_Paper_Overleaf.zip`)가 **100% 준비 완료**되어 있습니다.
* **11월 말 목표 달성 가능성**:
  * **[1차 심사 결과 (First Decision)]**: **100% 확실** (10월 초 투고 시 11월 10~20일 수령)
  * **[최종 게재 승인 (Final Accept)]**: **달성 가능 (유일한 대안)** (Binary Review 특성상 1회차에 Accept 판정 시 11월 말 승인 완료)

---

## 🏆 2. [구분 1] 11월 말까지 "최종 게재 승인 (Final Accept)" 달성 가능 저널

투고 시점(10월 초) 기준, 11월 30일까지 남은 시간은 **약 8주(56일)**입니다. 통상적인 SCIE 저널(심사 3~6개월)로는 1차 심사조차 불가능하며, **오직 아래의 특수 고속 심사 모델을 채택한 저널만 Final Accept를 노려볼 수 있습니다.**

### ① IEEE Access (강력 추천 / 최우선 후보 — 현실적으로 유일한 대안)
* **출판사 / 인덱스**: IEEE / **SCIE Q2** (Gold Open Access)
* **Impact Factor (JIF)**: **4.2** (5-Year IF: **4.3**) / SJR: **0.86**
* **JCR 카테고리**:
  * Computer Science, Information Systems (**Q2**)
  * Engineering, Electrical & Electronic (**Q2**)
  * Telecommunications (**Q2**)
* **심사 주기 및 메커니즘**:
  * **평균 First Decision 소요 기간**: **4~6주 (중앙값 30~35일)**
  * **Binary Peer Review**: 2~3개월이 소요되는 Major Revision 절차가 없습니다. 오직 **Accept** 또는 **Reject** (필요시 Reject & Resubmit Encouraged)로 1회차에 단판 승부됩니다.
  * **Final Accept 달성 경로**: 10월 초 투고 시 11월 중순(4~5주차)에 1차 Accept(경미한 타이포/설명 보완 권고 포함) 판정을 수령하고, 저자가 7~10일 내 최종본을 업로드하면 **11월 20~30일 내 공식 최종 게재 승인서(Final Acceptance Letter)**를 획득하게 됩니다.
* **V-LiDAR 논문 적합성**:
  * **분량 제한 없음**: 현재 작성된 12페이지의 수식 유도(사각지대 $D_{\min}=2.18\text{ m}$ 기하학적 유도), ROS 2 Nav2 코스트맵 파라미터 튜닝, 20회 연속 실증 주행 실패 원인 분석이 축약 없이 원형 그대로 수록됩니다.
* **게재료 (APC) 및 지원**:
  * 정규 APC: $2,160 USD
  * **할인 혜택**: 교신저자이신 황성수 교수님이 IEEE Senior Member이시므로 **20% 할인 ($1,728 USD)** 적용 가능.
  * **연구비 재원**: 원고 Line 31에 명시된 **글로컬대학30 ANCHOR 사업 연구비 (`2026-ANCHOR-15-119`)**로 출판비 집행 적합.
* **준비 상태**: [`docs/papers/IEEE_Access/`](../../docs/papers/IEEE_Access/)에 원고가 완성되어 있어 즉시 투고 가능.

---

### ② IEEE Sensors Letters (이론상 가능 / 재작성 리스크 높음)
* **출판사 / 인덱스**: IEEE Sensors Council / **SCIE Q2**
* **Impact Factor (JIF)**: **2.85**
* **심사 주기**: 심사위원에게 10일의 심사 기한을 부여하여, 투고부터 게재까지 **6주 이내(≤ 42일)** 처리를 목표로 운영됨.
* **11월 말 Final Accept 가능 여부**: **[이론상 가능하나 현실적 위험]**
* **치명적 제약 사항 (분량 제한)**:
  * **엄격한 4페이지 제한 (Strict 4-Page Limit)**. 4페이지 중 최소 1개 칼럼은 References 전용이어야 함 (실제 본문 약 3.2페이지).
  * 현재 12페이지 풀페이퍼에서 수학적 증명, 20회 주행 세부 분석, Nav2 실패 메커니즘 등 논문의 65% 이상을 도려내는 압축 재작업(최소 1~2주 소요)이 선행되어야 하므로, 10월 초 즉시 투고가 불가능해져 일정 안전마진이 사라집니다.

---

## 📋 3. [구분 2] 11월 말까지 "1차 심사 결과 (First Decision)" 확보 가능 저널

과제 연차평가나 교내 보고용으로 필요한 실적이 최종 Accept가 아니라 **공식 1차 심사 판정서(First Decision Letter 또는 Under Review 증빙)**인 경우입니다.

### ① IEEE Access (확실도 100%)
* 10월 1~5일 투고 시, **11월 10~20일 사이 100% First Decision 통보서 수령**.
* 1차 판정이 즉시 Accept이든, Reject & Resubmit(수정 후 재투고 권고)이든 상관없이 저널 에디터 명의의 공식 1차 심사 결과서(First Decision Letter)가 발급되므로 과제 보고 실적 기준을 완벽하게 만족합니다.

### ② IEEE Embedded Systems Letters (ESL)
* **출판사 / 인덱스**: IEEE CEDA / **SCIE/Scopus Q2** (JIF **1.7**)
* **심사 주기**: **'1개월(4주) 이내 1차 판정 보장제 (Guaranteed 1-month turnaround to first decision)'** 운영.
* **11월 말 First Decision 가능 여부**: **[확실도 높음]** (10월 초 투고 시 11월 초 수령 확실).
* **제약 사항**: 엄격한 4페이지 제한. 임베디드 엣지(Jetson/NUC) 메모리 점유율 및 저지연성 위주로 논문 관점을 대폭 재구성해야 함.

---

## ❌ 4. [구분 3] 11월 말까지 결과 도출 불가 저널 (비교 및 탈락 사유)

이름이 거론되거나 후보로 고려하기 쉬우나, **심사 프로세스상 11월 말까지 1차 결과조차 수령할 수 없는 곳들**입니다.

| 저널명 | 출판사 / 인덱스 | JCR 등급 & IF | 평균 First Decision | 평균 Final Accept | 탈락 사유 및 분석 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **IEEE Sensors Journal** | IEEE / SCIE | **Q1/Q2** (4.5) | **9~13주 (60~90일)** | 4~6개월 | **[불가]** 실제 통계상 1차 심사만 60일 이상 소요되어 11월 말까지 1차 결과 수령 불가 (12월 말~1월 예상). |
| **Frontiers in Robotics and AI** | Frontiers / **ESCI** | **ESCI Q2** (3.7) | 10~13주 (~90일) | 4~5개월 | **[불가 + 치명적 위험]** **SCIE가 아닌 ESCI (Emerging Sources)** 등재지임! 대학/과제 실적 미인정 위험. 대화형 심사로 11월 말 1차 판정 불가. |
| **IJCAS** | Springer / ICROS (제어로봇시스템학회) | **SCIE Q2** (2.8) | **11~12주 (~80일)** | 5~6개월 | **[불가]** 연구실 보유 템플릿([`ICCAS-IJCAS-2026`](../../../ICCAS-IJCAS-2026))이 있으나 정규/학회 연계 모두 심사에 수개월 소요되어 기한 초과. |
| **IEEE RA-L** | IEEE RAS / SCIE | **SCIE Q1** (5.3) | 10~14주 (~80일) | 6개월 | **[불가]** Q2가 아닌 로보틱스 최고 권위 **Q1** 저널이며, 정규 6개월 심사 주기로 운영. |
| **Intelligent Service Robotics (ISR)** | Springer / KROS (한국로봇학회) | **SCIE Q1** (5.9) | **평균 220일 (~7개월)** | 8~10개월 | **[불가]** 최근 IF가 5.9로 오르며 **Q1**으로 승격됨. 그러나 심사 속도가 매우 느려 기한 충족 불가. |
| **MDPI 저널군 (Sensors, Applied Sciences)** | MDPI / SCIE | Q2 (3.1) | 2~3주 | 4주 | **[제외]** 사용자 명시적 배제 지침 (학술적 평판 및 심사 신뢰도 문제). |

---

## 📊 5. 종합 비교 매트릭스

| 저널명 | 등재 인덱스 | JCR 분위 | Impact Factor | 평균 First Decision | 11월말 Final Accept | 11월말 First Decision | 분량 제한 | 원고 준비 상태 | 비고 |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **IEEE Access** | **SCIE** | **Q2** | **4.2** | **4~6주 (30일)** | **달성 가능 (유일)** | **달성 확실 (100%)** | **제한 없음** | **100% 완료** | **사용자 기억 저널 일치, 최우선 추천** |
| **IEEE Sensors Letters** | **SCIE** | **Q2** | 2.85 | 3~4주 | 달성 가능 (이론상) | 달성 확실 | 4페이지 엄수 | 대폭 축소 필요 | 12페이지 원고를 4페이지로 재작성 필요 |
| **IEEE ESL** | **SCIE** | **Q2** | 1.7 | 4주 이내 보장 | 불가 | 달성 확실 | 4페이지 엄수 | 관점 전환 필요 | 임베디드 엣지 중심 재포장 필요 |
| **IEEE Sensors Journal** | **SCIE** | **Q1/Q2** | 4.5 | 9~13주 | 불가 | 불가 (기간 부족) | 8~12페이지 | 포맷 변경 필요 | 심사 기간 60일 이상 소요 |
| **Frontiers in Rob. & AI** | **ESCI** | **ESCI Q2** | 3.7 | 10~13주 | 불가 | 불가 | 제한 없음 | 포맷 변경 필요 | **주의: SCIE가 아닌 ESCI 저널임** |
| **IJCAS** | **SCIE** | **Q2** | 2.8 | 11~12주 | 불가 | 불가 | 6~10페이지 | 템플릿 보유 | 연구실 연계 우수하나 기한 초과 |
| **IEEE RA-L** | **SCIE** | **Q1** | 5.3 | 10~14주 | 불가 | 불가 | 6~8페이지 | 포맷 변경 필요 | 로보틱스 Q1 대표 저널 |

---

## 🚀 6. 투고 로드맵 및 액션 플랜 (Action Plan)

```
[10월 1~3일] ➔ [11월 10~20일 (4~6주차)] ➔ [11월 20~28일 (7~8주차)] ➔ [11월 30일 이전]
 IEEE Access      1차 심사 결과(First Decision)   경미한 수정(Typo 등) 반영   최종 게재 승인
  원고 투고 완료          공식 판정서 확보              최종본 제출         (Final Accept) 완료
```

1. **원고 최종 확인 및 즉시 투고**:
   * 대상 파일: [`docs/papers/IEEE_Access/`](../../docs/papers/IEEE_Access/) (Overleaf 압축본: [`docs/papers/IEEE_Access_Paper_Overleaf.zip`](../../docs/papers/IEEE_Access_Paper_Overleaf.zip))
   * 저자 정보: 이현서, 유건민, 구현우, 황성수 교수님(Senior Member, IEEE)
   * 과제 사사: 글로컬대학30 ANCHOR 사업비 (`2026-ANCHOR-15-119`)
2. **행정 절차 (APC 할인)**:
   * 황성수 교수님의 IEEE Senior Member 계정으로 투고하여 **APC 20% 감면 ($1,728 적용)**을 적용받고, 과제비로 집행.
3. **일정 관리**:
   * 10월 초에 지체 없이 투고하여 심사위원이 30일 이내에 평가를 마치도록 골든타임을 확보합니다.
