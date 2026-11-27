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
| 제출 파일 | 투고 형식 PDF와 편집 가능한 LaTeX 소스·그림·서지 파일을 준비한다. 소스 ZIP의 파일을 한 디렉터리로 풀어 독립 컴파일을 확인한다. |

[JINT 공식 투고 지침](https://link.springer.com/journal/10846/submission-guidelines) · [Springer 사전 제출 체크리스트](https://link.springer.com/pre-submission?journalId=10846) · [Springer LaTeX 안내](https://www.springernature.com/la/authors/campaigns/latex-author-support)

## 원고에 남은 항목

| 표시 | 원고 위치 | 완료 기준과 참고 자료 |
| --- | --- | --- |
| **R1 현장 주행** | Evaluation protocol · Formal navigation outcomes | 4조건×2배치의 회차별 기록을 만든다. 표시한 목표 구역의 실제 도달, 접촉, 수동 개입, 제한시간 판정을 남기고 같은 회차의 rosbag과 결과 JSON을 연결한다. 실패와 자료 누락도 사유와 함께 기록한다. [현장 CSV 양식](../../paper/review/planning/templates/field_log.csv) · [기록 절차](../analysis/README.md#현장-csv의-자동-기록과-집계) |
| **R2 실차 설정** | System and method · Evaluation protocol | 출발 전 이전 목표 종료, 정지 시 속도·odom, 리셋 후 다음 회차 시작을 실차에서 확인한다. 사용한 코드·펌웨어, 모델·LUT 해시, 키 파라미터, 카메라 설치 높이·기울기와 가상 스캔 TF를 기록한다. 목표 구역 크기와 제한시간을 정하고 장애물 위치를 실측한다. [설정 기록 도구](../analysis/trial_records.py) · [기존 PC 스냅샷](../analysis/evidence/robot_checkout_snapshot_2026-11-26.json) · [기록된 스캔 TF](../analysis/evidence/scan_frame_transforms.csv) |
| **R3 학습 데이터** | Floor segmentation and scan generation | 원본 `data.yaml`이나 내보낸 데이터셋에서 출처·버전, 라벨 정의, train/val/test 수를 복원한다. 체크포인트의 클래스 이름과 배포 코드에서 쓰는 ID를 원본 라벨과 실제 추론 화면으로 대조한다. 검증 지표를 인용할 때 평가 분할의 확인 상태를 명시한다. [체크포인트 메타데이터](../../experiments/logs/pc_extracted/yolo11n_seg_checkpoint_meta.json) · [기존 자료 대조](../../paper/review/10_IJCAS_Manuscript_Repository_Check_20261118.md) |
| **R4 결과와 해석** | Formal navigation outcomes · Discussion · Abstract · Conclusion | 조건·배치별 전체 시도, 성공/판정 가능 시도, 95% Wilson 구간, 무효·판정 보류를 보고한다. 대표 성공 및 실패 회차가 있으면 RGB·scan·경로·odom을 시각에 맞춰 실패 지점을 확인한다. 판정 근거가 모자라면 원인 미상으로 남긴다. 결과에 맞춰 초록과 결론의 수치 및 주장 범위를 갱신한다. [결과 집계 코드](../analysis/summarize_trials.py) · [기존 파일럿 사건표](../analysis/evidence/pilot_nav_events.csv) |
| **R5 저자 선언** | Statements and Declarations | IEEE 원고의 저자 6명과 순서를 적용했다. 교신저자는 황성수 교수님이며, 전원 동등 기여 문구를 넣었다. JINT 연구에 해당하는 지원 과제와 이해충돌 선언을 확인해 채운다. [IEEE Access 원고](../../paper/workspace/IEEE_Access/main.tex) |
| **R6 자료 공개** | Data availability | 회차별 결과, 설정, 집계 코드, 공개 가능한 센서 기록의 보관 위치와 접근 조건을 정한다. RGB·외부 영상은 공개 가능한 범위를 확인하고 실제 접근 링크를 넣는다. [현재 rosbag 목록](../analysis/evidence/rosbag_inventory.csv) |
| **R7 선행 발표·투고 상태** | Introduction · 커버레터 | ICCAS 프로그램에 인용한 초록의 최종 공개 형태와 서지정보를 확인한다. IJCAS 등 관련 원고의 상태와 내용 중복·추가 범위를 공저자들과 확인해 커버레터를 마무리한다. [커버레터 초안](cover_letter_draft.md) |

## 현장 실험과 결과 반영

- **4조건:** 기존 장소 낮(`original_day`)과 밤(`original_night`), 인공조명 위주 실내(`indoor_artificial`), 자연광 구름다리(`bridge_daylight`). 각 조건에서 박스(`S1`)와 박스+의자(`S2`)를 사용한다. 현장 사진과 기록으로 시간대, 바닥, 통로 폭, 조명 상태를 설명한다. 장소 간 조명·바닥·공간 차이가 함께 바뀌므로 논문은 관측한 환경 조합에서의 결과를 해석한다. [기존 현장 계획](../../paper/review/planning/2026-11-26_plan.md)
- **평가값:** 시작점에서 목표 지점까지의 직선 변위는 10 m다. 제한시간 내 실제 목표 구역에 도달하고 접촉과 수동 개입이 없었던 회차를 성공으로 집계한다. 관찰자 또는 외부 영상이 판정 근거이며, Nav2 상태와 odom 경로는 진단에 쓴다. 최초 파일럿에서 로봇 PC의 통합 러너로 RGB·scan·odom이 저장되는지도 함께 확인한다. [파일럿 현장 로그 예시](../analysis/evidence/pilot_field_log.csv)
- **추가 거리·물리 LiDAR 비교를 할 때:** 현 원고의 핵심 실험은 구조화된 실내에서의 실제 장애물 회피다. 거리 정확도나 물리 LiDAR 대비 성능을 주장하기로 하면, 동일 좌표계의 실측 거리나 동일 주행 조건의 센서 비교를 설계한다. 기존 거리 비교는 기준 원점을 바꾸면 참조 거리부터 0.51–0.55 m 달라져, 원점 민감도 설명에 사용했다. [거리 참조 민감도](../analysis/evidence/scan_origin_reference_sensitivity.csv)

## 현재 초안에 반영된 근거

- 구름다리 파일럿의 주행 궤적과 Nav2 완료·계획 실패·속도 피드백 사례가 있다. 당시 완료 상태는 탐색 결과로 다루며, 본 실험 성공률에는 현장 판정과 본 실험 프로토콜을 충족한 회차를 사용한다. [파일럿 궤적](../manuscript/figures/bridge_pilot_odom.pdf) · [속도와 odom](../manuscript/figures/pilot_command_odometry.pdf)
- 같은 RGB 프레임에서 단안 방법의 CPU 처리 시간, 두 고정 장면에서의 시간 필터 켜기/끄기 결과, 파일럿 scan 발행률을 정리했다. 각 결과는 처리 시간, 스캔 안정성, 발행률을 설명하는 데 사용한다. [CPU 비교](../analysis/evidence/pytorch_comparison_summary.csv) · [시간 필터 재생 실험](../analysis/evidence/temporal_filter_replay.csv) · [스캔 기록 분석](../analysis/evidence/scan_recorder_timing.csv)

## 빌드

확인된 현장 CSV를 준비한 뒤 `JINT/`에서 실행하면 본 실험 표가 원고에 들어간다. 설정이 여러 개면 `--configuration`으로 표에 사용할 설정을 지정한다.

```bash
python3 analysis/summarize_trials.py ~/data/jint/field_log.csv \
  --latex-output manuscript/formal_trial_rows.tex
bash manuscript/build_manuscript.sh
```

`bash manuscript/build_manuscript.sh ~/data/jint/field_log.csv`로 조건별 결과표와 성공률·95% Wilson 구간 그래프를 함께 만든다. `~/data/jint/field_log.csv`에 설정이 여러 개면 `--configuration ID`를 덧붙인다. 판정 가능한 시도가 없는 조합은 그래프에서 추정치 없이 표시된다.

실험을 추가할 때는 `manuscript/main.tex`의 Formal navigation outcomes에 성공과 실패 패턴을 서술하고, Discussion에 원인과 근거를 연결한다. 대표 궤적이나 RGB·scan·경로·odom 시간축을 그림으로 만들면 `manuscript/figures/`에 넣고 결과에서 인용한다. 마지막에 Abstract, Conclusion, 커버레터의 수치와 분모를 대조한다. 완료한 `\pending`은 그 지점의 실제 내용으로 교체한다. 주황색 표시와 검토표는 내부 검토본에 표시된다.
