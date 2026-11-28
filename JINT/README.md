# JINT

**2026-11-28 원고는 추가 주행 없이 확보한 자료로 작성했다.** 중심 질문은 카메라가 본 장애물 경계가 어떤 스캔과 로컬 경로로 이어지고 어느 구간에서 관측을 신뢰할 수 있는가다. 팔각정 20회, 구름다리 20회, 5열람실 보존 17회의 Nav2 결과를 정지 규칙별로 보고한다. 별도 박스 접근 영상 두 개에서는 V-LiDAR·MiDaS·Depth Anything V2가 장애물 접근을 얼마나 일관되게 나타내는지 같은 프레임으로 비교했다. [투고 형식 PDF](JINT_first_draft.pdf) · [수치 및 재현 자료](analysis/evidence/ESM_2.zip)

**투고 판단.** 로봇 센싱과 이동 로봇 시스템에서 관측·제어를 연결한 실증 연구로 제출할 근거가 있다. 심사에서 가장 약한 부분은 동일 경로의 폐루프 센서 비교와 독립적인 실제 도달·접촉 판정이다. 따라서 57회를 합친 회피 성공률, 물리 LiDAR 대체 성능, 조명에 대한 일반화는 주장하지 않는다. 연구비·이해충돌·자료 접근·선행 투고 상태는 투고 전 교신저자가 확인한다. [JINT 연구 범위](https://link.springer.com/journal/10846/aims-and-scope) · [공저자 체크리스트](planning/JINT_draft_checklist.md)

팔각정 20회의 목표 활성 구간을 겹친 그림에는 완료 18회와 시간 초과 2회의 경로가 모두 들어 있다. 이 그림은 10 m 목표에 대한 Nav2 결과와 바퀴 odom 경로를 보여준다. 개별 회차의 박스 위치와 접촉 여부는 미확인이다. [전 회차 궤적](manuscript/figures/hall_goal_paths.pdf) · [회차별 횡이동](analysis/evidence/hall_goal_excursions.csv)

- [기존 rosbag으로 작성한 원고의 제출 전 체크리스트](planning/JINT_draft_checklist.md)
- [추가 비교 실험이 필요할 때의 계획](planning/2026-11-27_JINT_completion_plan.md)
- [11월 27일 팀 인수인계와 현장 체크리스트](planning/2026-11-27_plan.md)
- [11월 27일 구름다리 20회 결과와 코드 점검](planning/2026-11-27_bridge_review.md)
- [11월 27일 5열람실 17개 rosbag 결과와 odom 점검](planning/2026-11-27_class5_review.md)
- [JINT 투고 형식 초안 PDF](JINT_first_draft.pdf) · [Springer Nature LaTeX 양식](template/springer-nature-latex-2024-12.zip)
- [원고 소스와 그림](manuscript/) · [소스 ZIP](manuscript/JINT_draft_source.zip)
- [이전 4조건 원고 백업 PDF·LaTeX·보충자료](이전버전/2026-11-27_JINT_four_condition_backup.zip)
- [주석이 있는 내부 검토본 PDF](planning/JINT_author_review.pdf) · [이전 투고 전략](planning/2026-11-27_JINT_submission_strategy.md) · [커버레터](planning/cover_letter_draft.md)
- [실험 분석과 재현 방법](analysis/README.md) · [rosbag RGB로 논문 사진 만들기](analysis/README.md#실험-환경-사진과-기존-원고-그림) · [보충 그림 PDF](analysis/evidence/ESM_1.pdf) · [수치 근거 ZIP](analysis/evidence/ESM_2.zip)

현재 원고는 팔각정·구름다리·5열람실의 기록과 오프라인 재생을 사용한다. `JINT/`에서 `bash manuscript/build_manuscript.sh`로 초안 PDF, 공저자 검토 PDF와 소스 ZIP을 갱신한다. 회차별 현장 판정 자료가 추가되는 경우 같은 빌드에 현장 CSV를 인수로 줄 수 있다.
