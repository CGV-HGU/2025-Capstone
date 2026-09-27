# 📚 CGV 연구실 IJCAS 논문 레퍼런스 가이드
(International Journal of Control, Automation and Systems - Lab Publication Lineage)

본 디렉터리는 CGV 연구실(Computer Graphics & Vision Lab, 지도교수: 황성수 교수님)에서 **제어로봇시스템학회(ICROS)** 및 **Springer**가 공동 발간하는 SCIE 국제 학술지인 **IJCAS (International Journal of Control, Automation and Systems)**에 게재/제출했던 연구 논문 목록과 BibTeX 레퍼런스입니다.

---

## 📌 IJCAS 게재 논문 목록

### 1. Fast and Accurate Self-calibration Using Vanishing Point Detection in Manmade Environments (2020)
* **저자**: Sang Jun Lee, Sung Soo Hwang*
* **저널**: *International Journal of Control, Automation and Systems*, Vol. 18, No. 10, pp. 2609–2620, 2020.
* **DOI**: [10.1007/s12555-019-0284-1](https://doi.org/10.1007/s12555-019-0284-1)
* **연구 핵심**: 실내 인공 환경의 소실점(Vanishing Point) 검출을 활용한 빠르고 정밀한 카메라 셀프 캘리브레이션 기법.
* **V-LiDAR와의 연관성**: 본 V-LiDAR 시스템의 핵심 기반인 카메라 파라미터(높이 $H$, 틸트각 $\theta$, 기하학적 LUT 매핑)와 3D 기하 변환의 이론적 기초를 제공함.

### 2. Real-time Depth Estimation Using Recurrent CNN with Sparse Depth Cues for SLAM System (2020)
* **저자**: Sang Jun Lee, Heeyoul Choi, Sung Soo Hwang*
* **저널**: *International Journal of Control, Automation and Systems*, Vol. 18, No. 1, pp. 206–216, 2020.
* **DOI**: [10.1007/s12555-019-0350-8](https://doi.org/10.1007/s12555-019-0350-8)
* **연구 핵심**: SLAM 시스템에서 희소한 깊이 단서(Sparse Depth Cues)와 Recurrent CNN을 결합한 실시간 단안 깊이 추정.
* **V-LiDAR와의 연관성**: 단안 카메라 환경에서 고가의 거리 측정 센서 없이 깊이/거리 정보를 실시간으로 복원하는 연구 계보를 공유함.

### 3. Bag of Sampled Words: A Sampling-based Strategy for Fast and Accurate Visual Place Recognition in Changing Environments (2019)
* **저자**: Sang Jun Lee, Sung Soo Hwang*
* **저널**: *International Journal of Control, Automation and Systems*, Vol. 17, No. 10, pp. 2597–2609, 2019.
* **DOI**: [10.1007/s12555-018-0790-6](https://doi.org/10.1007/s12555-018-0790-6)
* **연구 핵심**: 동적 환경 변화에 강인한 샘플링 기반 장소 인식(Place Recognition) 및 루프 클로징 알고리즘.
* **V-LiDAR와의 연관성**: 실내 자율주행 모바일 로봇의 Visual SLAM(`stella-vslam`) 및 위치 추정 안정성 연구와 긴밀히 연계됨.

---

## 🏛️ 관련 제어로봇시스템학회 (ICROS) 국내 논문지 논문

* **실내 자율비행 드론의 장애물 회피 알고리즘 (2017)**:
  * 이혁진, 황성수, *"Obstacle Avoidance Algorithm for Indoor Autonomous Drone Using IR Sensor and Forward Image Information"*, *제어로봇시스템학회 논문지 (Journal of Institute of Control, Robotics and Systems)*, Vol. 23, 2017.
* **실내 영상 지도 생성 기법 (2017)**:
  * 황성수, 김혁민, *"A Robust Indoor Imagery Map Generation for Simple Backgrounds and Rotations by Separating the Intersection"*, *제어로봇시스템학회 논문지 (Journal of Institute of Control, Robotics and Systems)*, Vol. 23, 2017.

---

## 💡 연구 논문 인용 (BibTeX)
연구실 이전 연구와의 연구적 연속성(Lineage) 및 기여도를 강조하기 위해 `ijcas_lab_papers.bib` 파일에 정리되어 있으며, 본 논문 작성 시 `references.bib`에 쉽게 포함하여 인용할 수 있습니다.
