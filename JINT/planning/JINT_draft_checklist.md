# JINT 논문 초안 체크리스트

2026-11-27 KST · 내부 검토용

[투고 전략](2026-11-27_JINT_submission_strategy.md) · [투고 형식 초안 PDF](../JINT_first_draft.pdf) · [내부 검토본 PDF](JINT_author_review.pdf) · [LaTeX 원고](../manuscript/main.tex) · [소스 ZIP](../manuscript/JINT_draft_source.zip) · [커버레터 초안](cover_letter_draft.md)

새 실험에서 먼저 할 일은 **실차 세팅 기록(R2) → 4조건 주행과 현장 판정(R1) → 결과표·실패 분석·초록·결론 반영(R4)**이다. 내부 검토본의 검토표에서 해당 절로 이동할 수 있고, 본문의 주황색 `R1`–`R7` 표시를 누르면 검토표로 돌아온다. 11월 27일 로봇 PC에서 본 실험용 `~/data/jint/field_log.csv`와 원고에 들어갈 결과표를 찾지 못했다.

## JINT 투고 형식 확인

| 항목 | 확인 내용 |
| --- | --- |
| 페이지 제한 | JINT 공개 투고 지침과 사전 제출 체크리스트에서 일반 연구논문의 최대 페이지 수를 찾지 못했다. 제출 포털에 로그인한 뒤 나타나는 입력 조건은 교신저자가 제출 시 확인한다. |
| 원고 양식 | JINT가 권장하는 Springer Nature 공통 `sn-jnl` 템플릿의 공식 ZIP과 저장소의 클래스·번호 인용 스타일이 일치한다. 공개된 출판 PDF에는 출판 단계의 조판이 적용된다. |
| 첫 페이지 | 분야 번호 (3), (6), (7), 저자별 이름과 소속, 황성수 교신저자의 이메일, MSC 분류 68T40·68T45를 넣었다. 저자 사진·약력은 일반 첫 페이지 필수 항목에 없다. 각 저자의 ORCID는 확인된 경우에 추가한다. |
| 초록·키워드 | 초록 150–250단어, 키워드 4–6개. 현재 키워드는 6개이며 본 주행 결과를 반영한 후 초록 분량을 다시 확인한다. |
| 참고문헌·선언 | 본문은 대괄호 번호 인용을 사용한다. 참고문헌 뒤에 `Statements and Declarations`를 두었다. 연구비·이해충돌·자료 접근 문구는 확인된 내용으로 채우고 저자 기여 및 이해충돌은 포털에도 입력한다. |
| 제출 파일 | 투고 형식 PDF와 편집 가능한 LaTeX 소스·그림·서지 파일을 준비한다. 소스 ZIP을 풀어 독립 컴파일을 확인한다. 논문 제목·저널·저자·소속·교신 이메일이 들어 있는 [보충 그림 PDF](../analysis/evidence/ESM_1.pdf)와 [수치 근거 ZIP](../analysis/evidence/ESM_2.zip)은 각각 Online Resource 1과 2로 올린다. |

[JINT 공식 투고 지침](https://link.springer.com/journal/10846/submission-guidelines) · [Springer 사전 제출 체크리스트](https://link.springer.com/pre-submission?journalId=10846) · [Springer LaTeX 안내](https://www.springernature.com/la/authors/campaigns/latex-author-support)

IJCAS 원고의 ORCID 4개를 공개 등록 이름과 대조하고, Hyunmo Kang의 공개 ORCID는 이름과 한동대 소속을 확인해 제목 페이지에 연결했다. [Hyunwoo Gu](https://orcid.org/0009-0008-2779-648X) · [Gunmin Yoo](https://orcid.org/0009-0001-6453-8075) · [Hyunseo Lee](https://orcid.org/0009-0004-8217-7201) · [Hyunmo Kang](https://orcid.org/0009-0004-7846-649X) · [Sung Soo Hwang](https://orcid.org/0000-0002-0863-7503)

## 원고에 남은 항목

| 표시 | 원고 위치 | 완료 기준과 참고 자료 |
| --- | --- | --- |
| **R1 현장 주행** | Evaluation protocol · Formal navigation outcomes | 4조건×2배치의 회차별 기록을 만든다. 표시한 목표 구역의 실제 도달, 접촉, 수동 개입, 제한시간을 관찰해 판정한다. `trial_records.py add`로 현장 판정과 같은 회차의 rosbag·결과 JSON을 한 번에 등록할 수 있다. 실패와 자료 누락도 사유와 함께 기록한다. [현장 CSV 양식](../../paper/review/planning/templates/field_log.csv) · [기록 절차](../analysis/README.md#현장-csv의-자동-기록과-집계) |
| **R2 실차 설정** | System and method · Evaluation protocol | 출발 전 이전 목표 종료, 정지 시 속도·odom, 리셋 후 다음 회차 시작을 실차에서 확인한다. 사용한 코드·펌웨어, 모델·LUT 해시, 키 파라미터, 카메라 설치 높이·기울기와 가상 스캔 TF를 기록한다. 목표 구역 크기와 제한시간을 정하고 장애물 위치를 실측한다. [설정 기록 도구](../analysis/trial_records.py) · [기존 PC 스냅샷](../analysis/evidence/robot_checkout_snapshot_2026-11-26.json) · [기록된 스캔 TF](../analysis/evidence/scan_frame_transforms.csv) |
| **R3 학습 데이터** | Floor segmentation and scan generation | 배포 코드의 ID 0 설정은 복도·구름다리의 실제 기록 스캔과 재생 스캔을 대조했다. 원본 `data.yaml`이나 내보낸 데이터셋에서 출처·버전, 학습 라벨 정의, train/val/test 수를 확인한다. [체크포인트 메타데이터](../../experiments/logs/pc_extracted/yolo11n_seg_checkpoint_meta.json) · [배포 클래스 검증](../analysis/README.md#배포-클래스-검증) |
| **R4 결과와 해석** | Formal navigation outcomes · Discussion · Abstract · Conclusion | 조건·배치별 전체 시도, 성공/판정 가능 시도, 95% Wilson 구간, 무효·판정 보류를 보고한다. 대표 성공 및 실패 회차가 있으면 RGB·scan·경로·odom을 시각에 맞춰 실패 지점을 확인한다. 판정 근거가 모자라면 원인 미상으로 남긴다. 결과에 맞춰 초록과 결론의 수치 및 주장 범위를 갱신한다. [결과 집계 코드](../analysis/summarize_trials.py) · [기존 파일럿 사건표](../analysis/evidence/pilot_nav_events.csv) |
| **R5 저자 선언** | Statements and Declarations | IEEE 원고의 저자 6명과 순서를 적용했다. 교신저자는 황성수 교수님이며, 전원 동등 기여 문구를 넣었다. JINT 연구에 해당하는 지원 과제와 이해충돌 선언을 확인해 채운다. [IEEE Access 원고](../../paper/workspace/IEEE_Access/main.tex) |
| **R6 자료 공개** | Data availability | CPU 시간·스캔 갱신률·시간 필터의 수치 근거와 재계산 스크립트는 Online Resource 2에 있다. 본 주행의 회차별 결과, 설정과 공개 가능한 센서 기록의 저장 위치 및 접근 조건을 확인해 원고에 넣는다. [수치 근거 ZIP](../analysis/evidence/ESM_2.zip) · [현재 rosbag 목록](../analysis/evidence/rosbag_inventory.csv) |
| **R7 선행 발표·투고 상태** | Introduction · 커버레터 | ICCAS는 확인된 기술 프로그램을 인용했다. IEEE Access 거절 통지와 세 원고의 공유·추가 내용을 커버레터에 정리했다. ICCAS 개별 proceedings 수록과 IJCAS의 최종 처분은 확인 후 반영한다. [커버레터와 근거](cover_letter_draft.md#공저자-확인용) |

## 현장 실험과 결과 반영

- **4조건:** 기존 장소 낮(`original_day`)과 밤(`original_night`), 인공조명 위주 실내(`indoor_artificial`), 자연광 구름다리(`bridge_daylight`). 각 조건에서 박스(`S1`)와 박스+의자(`S2`)를 사용한다. 현장 사진과 기록으로 시간대, 바닥, 통로 폭, 조명 상태를 설명한다. 장소 간 조명·바닥·공간 차이가 함께 바뀌므로 논문은 관측한 환경 조합에서의 결과를 해석한다. [기존 현장 계획](../../paper/review/planning/2026-11-26_plan.md)
- **평가값:** 시작점에서 목표 지점까지의 직선 변위는 10 m다. 제한시간 내 실제 목표 구역에 도달하고 접촉과 수동 개입이 없었던 회차를 성공으로 집계한다. 관찰자 또는 외부 영상이 판정 근거이며, Nav2 상태와 odom 경로는 진단에 쓴다. 최초 파일럿에서 로봇 PC의 통합 러너로 RGB·scan·odom이 저장되는지도 함께 확인한다. [파일럿 현장 로그 예시](../analysis/evidence/pilot_field_log.csv)
- **추가 거리·물리 LiDAR 비교를 할 때:** 현 원고의 핵심 실험은 구조화된 실내에서의 실제 장애물 회피다. 거리 정확도나 물리 LiDAR 대비 성능을 주장하기로 하면, 동일 좌표계의 실측 거리나 동일 주행 조건의 센서 비교를 설계한다. 기존 거리 비교는 기준 원점을 바꾸면 참조 거리부터 0.51–0.55 m 달라져, 원점 민감도 설명에 사용했다. [거리 참조 민감도](../analysis/evidence/scan_origin_reference_sensitivity.csv)

## 현재 초안에 반영된 근거

- 구름다리 파일럿의 주행 궤적과 Nav2 완료·계획 실패·속도 피드백 사례가 있다. 당시 완료 상태는 탐색 결과로 다루며, 본 실험 성공률에는 현장 판정과 본 실험 프로토콜을 충족한 회차를 사용한다. [파일럿 궤적](../manuscript/figures/bridge_pilot_odom.pdf) · [속도와 odom](../manuscript/figures/pilot_command_odometry.pdf)
- 같은 RGB 프레임에서 단안 방법의 CPU 처리 시간, 고정 장면 두 건과 구름다리 주행 두 건의 필터 켜기/끄기 비교, 파일럿 scan 발행률을 정리했다. 이동 구간의 유효·무효 전환, 거리 변화량, 일시 반환의 보류와 확인 지연을 추가했다. [CPU 비교](../analysis/evidence/pytorch_comparison_summary.csv) · [시간 필터 재생 실험](../analysis/README.md#시간-필터-재생-실험) · [스캔 기록 분석](../analysis/evidence/scan_recorder_timing.csv)

## 그림 구성

| 그림 | 독자가 확인할 내용 | 본 실험 뒤 반영할 자료 |
| --- | --- | --- |
| **Figure 1 시스템 구성** | 실제 영상의 검출점에서 지면 투영·가상 스캔·Nav2 코스트맵으로 이어지는 경로와 차체 제어·odom 피드백 | 현재 시안을 기준으로 최종 편집 때 가독성과 조판을 다듬는다. 예시 RGB·scan·costmap은 같은 파일럿의 기록을 사용한다. [현재 Figure 1](../manuscript/figures/pipeline.pdf) |
| **Figure 4 실험 환경** | 기준 홀 낮·밤, 인공조명 실내, 자연광 구름다리의 바닥·통로·조명과 장애물 | 초안에 낮 세팅 사진과 복도·구름다리 파일럿 RGB를 표시했다. 본 실험 뒤 같은 박스 배치(`S1`)에서 우회 직전 박스와 바닥이 보이는 전방 RGB를 조건별로 넣는다. [사진 생성 방법](../analysis/README.md#실험-환경-사진과-기존-원고-그림) |
| **본 주행 결과 그림** | 조건×배치별 실제 회피 성능과 불확실성 | 성공 수/판정 가능 시도 수와 95% Wilson 구간을 표시한다. 전체 시도, 무효, 판정 보류 수는 결과표에서 확인한다. |
| **대표 주행 그림** | 실제로 어디서 우회했고 어떤 구간에서 멈췄는지 | 대표 성공과 실패를 RGB·odom 궤적으로 보여준다. 검출과 경로 생성의 순서가 판단에 중요하면 같은 구간의 scan·경로 시각을 함께 제시한다. [대표 주행 그림 코드](../analysis/render_trial_sequence.py) |

### Figure 1 최종 편집 계획

- 본 실험 그림이 들어온 뒤 섹션 제목, 패널 문자, 수치·단위와 사진 안 주석의 글꼴·굵기·정렬을 원고의 다른 그림과 맞춘다. 특히 지면 투영 도식과 코스트맵 안의 작은 주석을 우선 키운다.
- 강조색은 검출점과 같은 각도 채널을 연결하는 데 집중한다. 패널 테두리와 배경색을 정돈하고 화살표 굵기·머리 크기·끝점 여백을 맞춰 데이터 흐름과 점선 피드백을 쉽게 읽을 수 있게 한다.
- 원고에 삽입한 실제 폭에서 처음부터 읽어 본다. 검출점 → 투영 → 스캔 → 코스트맵 순서가 한눈에 이어지고 거리·각도·경로 표기가 또렷한지 화면과 흑백 출력에서 확인한다.

## 빌드

확인된 현장 CSV를 준비한 뒤 `JINT/`에서 실행하면 본 실험 표가 원고에 들어간다. 설정이 여러 개면 `--configuration`으로 표에 사용할 설정을 지정한다.

```bash
python3 analysis/summarize_trials.py ~/data/jint/field_log.csv \
  --latex-output manuscript/formal_trial_rows.tex
bash manuscript/build_manuscript.sh
```

`bash manuscript/build_manuscript.sh ~/data/jint/field_log.csv`로 조건별 결과표와 성공률·95% Wilson 구간 그래프를 함께 만든다. `~/data/jint/field_log.csv`에 설정이 여러 개면 `--configuration ID`를 덧붙인다. 판정 가능한 시도가 없는 조합은 그래프에서 추정치 없이 표시된다.

실험을 추가할 때는 `manuscript/main.tex`의 Formal navigation outcomes에 성공과 실패 패턴을 서술하고, Discussion에 원인과 근거를 연결한다. 현장 CSV의 회차 ID와 RGB 프레임 시각을 지정해 대표 주행의 영상·odom 그림을 만들면 빌드가 본문과 소스 ZIP에 넣는다. 마지막에 Abstract, Conclusion, 커버레터의 수치와 분모를 대조한다. 완료한 `\pending`은 그 지점의 실제 내용으로 교체한다. 주황색 표시와 검토표는 내부 검토본에 표시된다. [대표 주행 그림 생성](../analysis/README.md#대표-주행의-rgb와-궤적)
