# 계획표와 현장 기록 양식

이 폴더의 CSV는 **계획 또는 빈 양식**이다. 실제 실험 결과가 아니다. 프로토콜은 [01 문서](../01_Experiment_Setup_and_Three_Day_Plan.md)를 따른다.

| 파일 | 행 구성 | 사용법 |
| --- | --- | --- |
| [trial_plan.csv](trial_plan.csv) | 계획 60행 | E1 본 주행 A/B × S1/S2/S3 × 10회. `status=planned` |
| [static_reference_plan.csv](static_reference_plan.csv) | 계획 18행 | E2 A/B × 2.5/3.5/4.5 m × 재배치 3회. calibration 6, evaluation 12 |
| [field_log.csv](field_log.csv) | 헤더만 | 각 실제 시도에 새 행을 기록. 계획과 실제 결과를 분리 |
| [static_measurement_log.csv](static_measurement_log.csv) | 헤더만 | E2 실제 측정값·기준점·방법·클립·보정/평가 구분 기록 |
| [asset_manifest.csv](asset_manifest.csv) | 헤더만 | 원본·trial/clip·그림 패널·처리 설정을 연결 |

## 계획과 실제 시도

- `protocol_id=proposal_v1`은 미확정 제안 버전이다. 첫날 장소·장착·판정·실제 물체를 확정한 뒤 버전을 고정한다. 숫자를 사후 조정해 성공한 데이터만 포함하지 않는다.
- E1 `block_id` 하나에는 S1/S2/S3가 한 번씩 있다. 블록 내 순서는 Python 표준 `random.Random(20261113)`으로 생성했다. 실제 순서는 `field_log.actual_order`에 따로 남긴다.
- 배치 좌표는 **출발 base_link 지면 투영 기준 물체 중심**이다. 두 환경의 S2 물체는 동일한 실제 의자/카트로 선택하고 ID·크기를 별도 기록한다. E2 거리는 **센서 기준 접촉 전면의 수평 정답 거리**로, E1 중심 좌표와 정의가 다르다.
- `plan_id`는 계획 ID, `record_id`는 실제 시도 ID다. 동일 계획을 절차적 무효로 재시도해도 원래 행을 지우지 말고 새로운 `record_id`와 `attempt_index`를 쓴다. 충돌·timeout은 무효로 바꾸지 않는다.
- `phase`는 `pilot`, `main`, 별도 전후 실험 등의 구분이다. pilot과 본 평가, 변경된 configuration의 결과를 자동 합산하지 않는다.
- E2는 같은 자리의 연속 녹화 3개가 아니라 **세 번의 독립적인 물체 재배치·거리 확인**이다. placement 1은 보정, 2/3은 평가. 각 모델의 보정 파라미터는 보정 자료에서만 정하고 평가 전에 고정한다.

## 실제 결과와 자산 기록

- `outcome`: `success`, `collision`, `timeout`, `nav_abort`, `manual_intervention`, `invalid`. 복합 원인/개입은 별도 열과 notes에 모두 남기고 최종 분류 규칙을 프로토콜에 고정한다.
- `contact_observed`, `manual_intervention`, `evaluation_included`는 실제 확인 후 `true/false`를 채운다. 모르면 빈칸과 이유를 남긴다. 전체 시도·유효 시도·성공 분모를 구분한다.
- `elapsed_s`는 `(result_or_cancel_time_ns - goal_accepted_time_ns) / 1e9`다. 두 타임스탬프가 같은 시계 기준인지 확인하고, 로그 시계 차이가 있으면 방법/제한을 기록한다.
- `start_time_iso8601_with_offset`에는 시간대까지 남긴다. odom 경로 길이는 odometry 기반임을 명시한다. `external_goal_check`에는 외부 목표 확인 방법/결과를 기록한다.
- 정적 클립의 실제 거리·실제 취득 파일·측정 오차·보정 분할은 `static_measurement_log.csv`와 manifest에 연결한다. 계획 거리만을 실제 측정값으로 취급하지 않는다. `measurement_uncertainty_m`는 측정 방법/기기 근거가 있을 때 채우며 정밀도를 임의로 만들어 넣지 않는다.
- 자산은 `asset_id`를 통해 bag/영상/CSV 및 F01~F09, T01~T04와 연결한다. SHA256과 실제 원본 경로, timestamp, 좌표계, 선택 규칙, 처리 코드/설정을 기록한다. 결과물을 생성하지 않았으면 `output_path`를 비워 둔다.
- 비밀번호·토큰은 기록하지 않는다. 대용량 원본은 Git에 올리기보다 검증된 보관 경로와 파일 식별값을 연결한다.
