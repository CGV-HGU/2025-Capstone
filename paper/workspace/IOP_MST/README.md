# 📗 Measurement Science and Technology (IOP Publishing) 투고 가이드 및 워크스페이스

본 디렉터리는 **영국 물리협회(IOP Publishing)의 전통 권위 SCIE Q2 저널인 *Measurement Science and Technology (MST / MSAT)*** 투고를 위한 완성된 LaTeX 원고 및 제출 부속 자료 세트입니다.

---

## 📌 1. 저널 주요 정보 요약

* **저널 정식 명칭**: *Measurement Science and Technology* (MST, 연구실 통칭 MSAT)
* **출판사**: IOP Publishing (Institute of Physics)
* **등재 인덱스**: **SCIE Q2** (Instruments & Instrumentation / Engineering, Multidisciplinary)
* **Impact Factor (JIF)**: **2.7** (5-Year IF: **2.8**) / CiteScore: **4.7**
* **ISSN**: 0957-0233
* **출판 비용 (APC)**:
  - **구독 모델 (Subscription / Traditional)**: **$0 USD (완전 무료 출판)** ⭐
  - 오픈액세스 모델 (Gold OA, 선택사항): $2,930 USD
  - 👉 *Elsevier C&EE와 더불어 무료(APC $0)로 SCIE Q2 실적을 확보할 수 있는 최고 전통의 저널입니다.*
* **심사 기간**:
  - **평균 First Decision (1차 심사 결과)**: **4 ~ 6주**
  - 계측 및 센서 분야 전문 심사위원단의 빠른 피드백 운영.

---

## 📂 2. 폴더 내 구성 파일

* [`main.tex`](main.tex): IOP Publishing 표준 `iopart` 포맷으로 맞춤 변환된 전체 풀페이퍼 원고 (측정 오차 보정, 2D 유클리드 기하 보정 수식, 정적/동적 20회 측정 정밀도 분석 강조)
* [`cover_letter.txt`](cover_letter.txt): 저널 편집장(Editor-in-Chief) 제출용 맞춤 공식 커버레터
* [`references.bib`](references.bib): 참고문헌 BibTeX 데이터베이스
* [`figures/`](figures/): 고해상도 그림 및 실험 사진 폴더

---

## 🎯 3. IOP MST(MSAT) 투고 시 핵심 전략

1. **원고 프레이밍 (Framing as Optical Measurement)**:
   - 일반 로봇 제어보다는 **"단안 카메라를 통한 평면 2D 거리 측정 및 센서 에뮬레이션의 정밀도, 기하학적 캘리브레이션 오차 제거(18% 외곽 왜곡 보정), 광학적 사각지대(2.18m) 물리적 한계 규명"** 관점으로 서술되어 있어 저널의 Scope(계측, 센서 시스템)에 최적으로 부합합니다.
2. **Subscription Model 투고**:
   - 투고 시스템(ScholarOne)에서 출판 옵션 선택 시 "Subscription (Standard Publication)"을 선택하면 게재료가 **$0 (전액 무료)** 청구됩니다.
3. **Overleaf 즉시 컴파일**:
   - `iopart.cls`와 `amsmath` 간의 수식 매크로 충돌을 완벽히 방지하도록 프리앰블이 세팅되어 있어 Overleaf에서 바로 컴파일할 수 있습니다.
