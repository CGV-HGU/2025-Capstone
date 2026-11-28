# JINT 논문 초안 체크리스트

2026-11-28 KST · 내부 검토용

[현재 투고 판단](../README.md) · [투고 형식 초안 PDF](../JINT_first_draft.pdf) · [내부 검토본 PDF](JINT_author_review.pdf) · [LaTeX 원고](../manuscript/main.tex) · [소스 ZIP](../manuscript/JINT_draft_source.zip) · [커버레터 초안](cover_letter_draft.md)

현재 원고는 **기존 rosbag 분석과 동일 RGB 재생**을 연구 범위로 삼는다. 제출 전에는 기록의 출처·실행 설정·데이터 선언을 교신저자와 대조한다. 내부 검토본의 주황색 표시로 확인 위치를 찾을 수 있다.

## JINT 투고 형식 확인

| 항목 | 확인 내용 |
| --- | --- |
| 페이지 제한 | JINT 공개 투고 지침과 사전 제출 체크리스트에서 일반 연구논문의 최대 페이지 수를 찾지 못했다. 제출 포털에 로그인한 뒤 나타나는 입력 조건은 교신저자가 제출 시 확인한다. |
| 원고 양식 | JINT가 권장하는 Springer Nature 공통 `sn-jnl` 템플릿의 공식 ZIP과 저장소의 클래스·번호 인용 스타일이 일치한다. 공개된 출판 PDF에는 출판 단계의 조판이 적용된다. |
| 첫 페이지 | 분야 번호 (3), (6), (7), 저자별 이름과 소속, 황성수 교신저자의 이메일, MSC 분류 68T40·68T45를 넣었다. 저자 사진·약력은 일반 첫 페이지 필수 항목에 없다. 각 저자의 ORCID는 확인된 경우에 추가한다. |
| 초록·키워드 | 초록 150–250단어, 키워드 4–6개. 현재 키워드는 6개이며 제출 전에 초록과 키워드를 한 번 더 대조한다. |
| 참고문헌·선언 | 본문은 대괄호 번호 인용을 사용한다. 참고문헌 뒤에 `Statements and Declarations`를 두었다. 연구비·이해충돌·자료 접근 문구는 확인된 내용으로 채우고 저자 기여 및 이해충돌은 포털에도 입력한다. |
| 제출 파일 | 투고 형식 PDF와 편집 가능한 LaTeX 소스·그림·서지 파일을 준비한다. 소스 ZIP을 풀어 독립 컴파일을 확인한다. 논문 제목·저널·저자·소속·교신 이메일이 들어 있는 [보충 그림 PDF](../analysis/evidence/ESM_1.pdf)와 [수치 근거 ZIP](../analysis/evidence/ESM_2.zip)은 각각 Online Resource 1과 2로 올린다. |

[JINT 공식 투고 지침](https://link.springer.com/journal/10846/submission-guidelines) · [Springer 사전 제출 체크리스트](https://link.springer.com/pre-submission?journalId=10846) · [Springer LaTeX 안내](https://www.springernature.com/la/authors/campaigns/latex-author-support)

IJCAS 원고의 ORCID 4개를 공개 등록 이름과 대조하고, Hyunmo Kang의 공개 ORCID는 이름과 한동대 소속을 확인해 제목 페이지에 연결했다. [Hyunwoo Gu](https://orcid.org/0009-0008-2779-648X) · [Gunmin Yoo](https://orcid.org/0009-0001-6453-8075) · [Hyunseo Lee](https://orcid.org/0009-0004-8217-7201) · [Hyunmo Kang](https://orcid.org/0009-0004-7846-649X) · [Sung Soo Hwang](https://orcid.org/0000-0002-0863-7503)

## 원고에 남은 항목

| 표시 | 원고 위치 | 완료 기준과 참고 자료 |
| --- | --- | --- |
| **R1 기록과 판정** | Evaluation · Archived recordings and outcome measures | 기준 홀 20회·구름다리 추적 20회·5열람실 보존 17개의 회차 번호와 결과표를 대조한다. Nav2 결과·odom 목표 접근·실제 도달의 근거를 구분한다. [기준 홀 분석](../analysis/README.md#기존-원고-주행-결과의-재사용) · [5열람실 분석](2026-11-27_class5_review.md) |
| **R2 실차 설정** | System and method | 기록된 TF·카메라 보정·배포 모델 및 LUT·버전 자료와 본문을 맞춘다. 실행 설정이 확인되지 않는 회차는 출처가 확인된 참고 설정과 구분한다. [PC 스냅샷](../analysis/evidence/robot_checkout_snapshot_2026-11-26.json) · [기록된 스캔 TF](../analysis/evidence/scan_frame_transforms.csv) |
| **R3 학습 데이터** | Floor segmentation and scan generation | 배포 코드의 ID 0 설정은 복도·구름다리의 실제 기록 스캔과 재생 스캔을 대조했다. 원본 `data.yaml`이나 내보낸 데이터셋에서 출처·버전, 학습 라벨 정의, train/val/test 수를 확인한다. [체크포인트 메타데이터](../../experiments/logs/pc_extracted/yolo11n_seg_checkpoint_meta.json) · [배포 클래스 검증](../analysis/README.md#배포-클래스-검증) |
| **R4 결과와 해석** | Results · Discussion · Abstract · Conclusion | 구름다리 카트·5열람실 의자의 시각·스캔·코스트맵을 대조한다. 정체·복구의 사건 순서와 원인이 남은 사례를 구분한다. 초록과 결론을 각 세션의 지표에 맞춘다. [카트 기록](2026-11-27_bridge_review.md) · [의자 기록](../analysis/evidence/class5_20_chair_scan.csv) |
| **R5 저자 선언** | Statements and Declarations | IEEE 원고의 저자 6명과 순서를 적용했다. 교신저자는 황성수 교수님이며, 전원 동등 기여 문구를 넣었다. JINT 연구에 해당하는 지원 과제와 이해충돌 선언을 확인해 채운다. [IEEE Access 원고](../../paper/workspace/IEEE_Access/main.tex) |
| **R6 자료 공개** | Data availability | CPU 시간·스캔 갱신률·시간 필터의 수치 근거와 재계산 스크립트는 Online Resource 2에 있다. 세션별 수치와 공개 가능한 원본의 접근 조건을 확인해 원고에 넣는다. [수치 근거 ZIP](../analysis/evidence/ESM_2.zip) · [현재 rosbag 목록](../analysis/evidence/rosbag_inventory.csv) |
| **R7 선행 발표·투고 상태** | Introduction · 커버레터 | ICCAS는 확인된 기술 프로그램을 인용했다. IEEE Access 거절 통지와 세 원고의 공유·추가 내용을 커버레터에 정리했다. ICCAS 개별 proceedings 수록과 IJCAS의 최종 처분은 확인 후 반영한다. [커버레터와 근거](cover_letter_draft.md#공저자-확인용) |

## 근거와 그림

- **환경과 회피.** Figure 4의 세 장소 사진과 구름다리 카트·통 및 5열람실 의자 기록을 사용한다. 5열람실 의자 아래 픽셀, 가상 스캔, 코스트맵의 대응을 새로 확인했다. [세 장소 그림](../manuscript/figures/site_views_archive.pdf) · [의자 그림](../manuscript/figures/class5_chair_scan.pdf)
- **정체와 복구.** 5열람실 20번에서 초기화 요청과 움직임 재개 시각을 대조했다. 19번의 정지 진단은 보고된 바퀴 위치와 odom이 함께 기록된 구간으로 제한한다. [복구 수치](../analysis/evidence/class5_20_recovery_costmap.csv)
- **계산·필터.** 같은 영상에서 CPU 시간과 세 방법의 박스 접근 신호 순서를 비교하고, 필터 전후의 거리 변화·유효 전환·확인 지연을 보고한다. [수치 ZIP](../analysis/evidence/ESM_2.zip)

## 제출 파일 확인

1. `JINT/`에서 `bash manuscript/build_manuscript.sh`로 초안 PDF, 공저자 검토 PDF, LaTeX 소스 ZIP을 다시 만든다.
2. 본문 인용, 그림·표 참조, 수치 ZIP 안의 CSV, 소스 ZIP 독립 컴파일 결과를 확인한다. 최종 투고 파일에는 투고 형식 PDF·소스 ZIP·두 보충자료·커버레터를 사용한다.
3. 교신저자가 최종적으로 연구비, 이해충돌, 공저자 승인, IJCAS 상태 및 원자료 접근 방법을 확인한다.
