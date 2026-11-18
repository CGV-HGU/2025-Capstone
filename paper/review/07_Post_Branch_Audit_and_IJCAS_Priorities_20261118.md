# post 재점검과 IJCAS 실험·피규어 우선순위

- 점검일: 2026-11-18
- 기준: `post`, `5add6fb` (`2026-11-13`), `git pull --ff-only origin post` 결과 `Already up to date.`
- 검증: 저장소 소스·리뷰·원고·추출 CSV, 원본 함수에 합성 입력을 제공한 오프라인 확인. 로봇 접속·구동, 실제 모델 추론, ROS 통합 실행은 수행하지 않았다.
- 기존 기준 문서: [교수님 합의 범위](06_Advisor_Agreed_Scope_and_Next_Steps.md), [실험 계획](planning/01_Experiment_Setup_and_Three_Day_Plan.md), [피규어 계획](planning/02_Figure_and_Table_Plan.md).
- 재현 자료: [확인 스크립트](verification/20261118_post_branch_audit.py), [출력 JSON](verification/20261118_post_branch_audit.json). 합성 입력의 결과는 새로운 실험 성능으로 사용하지 않는다.

> **후속 자료 반영:** 11/16 미팅 PDF에는 물리 2D LiDAR 비교가 추가 실험 **후보**로 명시돼 있다. 이후 사용자는 2D 장비가 없다는 점과 IJCAS AE 원문(`LiDAR or alternative baseline`)을 제공했다. 현재 우선순위는 [IJCAS 코멘트와 3D LiDAR 활용 판단](09_IJCAS_Review_Priority_and_3D_LiDAR_Option_20261118.md)을 따른다. [미팅 분석](08_Week12_Meeting_Analysis_and_Updated_Experiment_Strategy_20261118.md)의 물리 비교안은 선택지이며 필수가 아니다. 소스·CSV 검증 결과는 그대로 유효하다.

## 1. 판단과 실행 순서

현재는 기존 60회 계획을 시작하기 전에 **평가 코드의 측정 유효성, 실제 실행 버전, 결과 판정·녹화 경로**부터 고정할 필요가 있다. 원고의 수치와 현재 재현되는 집계가 다르고, 실제 실행 경로와 오프라인 benchmark 경로도 다르다.

권장 순서는 다음과 같다.

1. 실제 제출 PDF와 이전 리뷰 원문을 확정하고, 이번 논문의 주장과 검증 범위를 고정한다.
2. 바닥 클래스 선택, 입력 시각·만료, baseline의 무검출·실패 처리와 metric 보정을 정리한다.
3. 작은 pilot으로 RGB/마스크/scan/주행 상태/목표 결과가 함께 기록되는지 확인한다.
4. 기존안인 E1 60회 주행과, baseline 근거가 복구되지 않으면 E2 18클립을 취득한다.
5. 모든 시도를 집계하고, 원자료에서 그림·표와 리뷰 대응표를 만든다.

초기 범위 판단은 11/13 문서를 따라 물리 LiDAR, 장착 높이·틸트 sweep, 움직이는 보행자, 임베디드 보드 검증을 보류하는 것이었다. 이후 확인한 **11/16 미팅 자료는 물리 2D LiDAR 비교를 후보에 추가**했으므로 해당 항목은 [08 갱신안](08_Week12_Meeting_Analysis_and_Updated_Experiment_Strategy_20261118.md)에 따라 준비·범위를 재검토한다. 나머지 항목을 모두 필수로 확대할 근거는 아직 없다. 기여와 결론은 **확인한 운용 범위에서의 단안 scan 합성 및 Nav2 통합**에 맞추고, LiDAR 수준의 일반적 신뢰성·대체 가능성은 실제 비교 근거에 따라 기술한다.

## 2. 확보된 자료와 출처 확인이 필요한 자료

### 이미 저장소에 있는 자료

- [Access 판정문과 3명 리뷰 원문](01_IEEE_Access_Decision_and_Raw_Reviews.md): 2026-11-12 판정 및 구체적 리뷰가 있다. 이 기록에 대해서는 빠른 탈락을 근거로 외부 리뷰 전 desk rejection이라고 부르면 안 된다.
- 교수님 합의 범위, 60회 주행 계획표, 18클립 정적 측정 계획표, 현장 기록 양식, F01~F09/T01~T04 계획이 이미 있다. 계획표의 상태는 `planned`이며 완료된 결과가 아니다.
- 인벤토리 CSV 40행: Active 28, Trash 12, 카메라 포함 bag 9개, 카메라 메시지 합계 30,706개. 압축 손상으로 기록된 항목 1개가 있다. 이번에는 CSV를 재집계한 것이며 원본 bag을 새로 열어 확인한 결과는 아니다.
- 20회 주행 추출 CSV, benchmark 입력 CSV 2개(각 100행), 학습 100 epoch 기록, 모델의 `person/floor` 클래스 정보와 일부 학습 설정이 있다.
- 기존 단문 리뷰는 `docs/reviews/comment.txt`와 `paper/archive/submitted_papers/iccas/ICCAS_Reviewer_Comments.txt`에 있다. 내용은 단일 환경·단일 박스, 22% 실패, 광택/조명 민감도, 비교 기법 부재를 지적한다.

### 추가 확인이 필요한 정보

| 정보 | 현재 부족한 점 | 확보 방법/용도 |
| --- | --- | --- |
| IJCAS AE 리뷰 | **사용자가 11/18 원문과 출처를 확인해 제공함.** 기존 파일은 ICCAS라는 경로에 보관돼 있었음 | [제공 원문과 해석](09_IJCAS_Review_Priority_and_3D_LiDAR_Option_20261118.md)을 대응표의 근거로 사용 |
| 실제 Access 최종 제출본 | `paper/workspace/IEEE_Access/main.tex`, 날짜가 붙은 원고, `docs/papers/IEEE_Access/main.tex`의 해시가 서로 다르다 | 제출 시스템의 최종 PDF/소스와 매칭해 고정 |
| 원본 RGB bag·실험 메타데이터 | 이번 저장소 검색에서 `.db3`, `.db3.zstd`, `.mcap`은 찾지 못했다. 기존 문서에는 로봇 PC에 남아 있다고 기록돼 있다 | 필요한 원본을 복사·검증하고 commit/모델/LUT/런타임 파라미터·장소·결과와 연결 |
| baseline `0.192 m`의 근거 | 현 CSV와 집계기로 재현되지 않는다 | 계산 스크립트, 대상 프레임, GT, 외인자, 보정 파라미터와 split, 변경 이력을 확보 |
| ablation 입력과 실행 기록 | 기록된 20회 자료의 인벤토리에는 영상 메시지가 없고, 구성별 결과의 실행 근거가 미확인 | 별도 RGB/마스크 입력과 구성별 출력 확인. 없으면 새로 평가하고 기존 수치는 재사용 보류 |
| 학습 데이터의 구성·분할·정답 | 체크포인트 설정은 있지만 원본 `data.yaml`과 독립 test 자료를 이번 검색에서 찾지 못했다 | 장소별 train/val/test 분리, annotation 기준, floor 전용 지표와 경계 품질 확인 |
| 현재 현장 제약 | 기존 3일은 제안 일정이다 | 현재 마감, 장소 2곳, 장비·관찰 인원·조명 기록·거리 측정 수단을 확정 |

## 3. 직접 확인한 핵심 문제

### 3.1. baseline 수치: 200개 입력 중 거리 오차 집계는 59개

[`run_dual_bag_benchmark.py:60`](../../experiments/run_dual_bag_benchmark.py#L60)의 조건을 CSV에 그대로 적용했다.

```python
robot_x < 3.8 and 2.18 <= gt_distance <= 5.0 and isfinite(vl_dist)
```

| 항목 | V-LiDAR | MiDaS | Depth Anything V2 |
| --- | ---: | ---: | ---: |
| 거리 오차 집계 행 수 | 59 | 59 | 59 |
| 재집계 MAE (m) | 1.529289 | 0.879600 | 3.298069 |
| 200행 지연 평균 (ms) | 19.722 | 42.930 | 601.377 |
| 200행 지연 모집단 표준편차 (ms) | 18.288 | 7.436 | 26.462 |

기존 `baseline_benchmark_report.md`와는 일치한다. 검토한 [원고:547](../workspace/IEEE_Access/main.tex#L547)의 V-LiDAR `0.192 m`, `12.9 ± 1.9 ms`와는 일치하지 않는다. 다른 버전·보정·백엔드 결과일 수 있으므로 해당 수치가 틀렸다고 확정하는 것이 아니라 **현재 저장소에서 재현 근거가 연결되지 않는다**고 판단한다.

또한 이것은 기존 `gt_distance` 열을 신뢰한 재집계다. 물리적 거리 정확도가 검증됐다는 뜻은 아니다. 기존 코드는 odometry의 x/y만 사용하고 yaw·센서 외인자·실제 관측 표적 대응을 반영하지 않는다. V-LiDAR는 녹화 `/scan`의 최소값을 쓰기도 하고 다른 방법은 현재 재추론의 중앙 구간 median을 사용한다. 같은 정의의 관측값으로 맞춰야 한다.

### 3.2. 측정 실패가 유효한 숫자로 변환될 수 있음

- [`benchmark_depth_baselines.py:336`](../../experiments/benchmark_depth_baselines.py#L336): 중앙 채널의 유효값이 하나도 없어도 `GT + 0.35`를 추정 거리로 반환한다. 실제 함수를 모두 `inf`인 입력으로 실행하니 GT 2.50 m에 대해 추정 2.85 m, 오차 0.35 m가 반환됐다.
- 같은 파일 677행 이후의 예외 경로에는 모델별 지연·FPS·오차 상수가 결과 목록에 추가된다. 이번 검토만으로 과거 표가 그 경로에서 만들어졌다고 단정할 수는 없지만, 후속 측정은 이 경로를 제거하거나 `not_measured`로 명확히 분리해야 한다.
- MiDaS의 `1000 / disparity`(207행), DA-V2의 프레임별 min/max → 0.5~5.5 변환(270행)은 원고가 기술하는 독립 보정 자료 기반 `(s,t)` 추정과 다르다.

**조치:** 무검출/모델 실패/GT 없음/범위 밖을 상태로 기록하고, 탐지 유효율과 유효 표적의 오차를 함께 보고한다. 모든 모델은 동일한 독립 GT·평가 구간·사전 고정한 보정 절차를 사용한다.

### 3.3. 온라인 바닥 마스크에 person이 합쳐질 수 있음

모델 메타데이터의 클래스는 `0: person`, `1: floor`다. 하지만 [`floor_detector.py:162`](../../src/freespace_detection/freespace_detection/floor_detector.py#L162)는 모든 마스크를 클래스 구분 없이 OR 병합한다.

원본 `image_callback`에 합성 마스크를 입력했다. 바닥에 90×60 픽셀 비바닥 영역을 두면 closing 후에도 비바닥 5,400픽셀이 남지만, 같은 영역을 채우는 person 마스크를 함께 입력하면 비바닥 픽셀이 0이 됐다. 이는 마스크 병합 경로의 결과이며 실제 사람 탐지율을 측정한 실험은 아니다.

**조치:** floor 클래스 선택과 floor 미검출 처리를 고정한 뒤 본 평가를 시작한다. benchmark wrapper도 floor를 찾지 못하면 모든 클래스로 fallback하는 경로가 있어 함께 점검해야 한다. 변경 전후 버전을 분리한다.

### 3.4. scan의 최신 시각이 최신 관측을 보장하지 않음

[`fake_lidar_with_tf.py:60`](../../src/fake_lidar_with_tf/fake_lidar_with_tf/fake_lidar_with_tf.py#L60)는 별도 10 Hz 타이머에서 마지막 거리 배열을 현재 시각으로 다시 발행한다. 상류 `Float32MultiArray`에는 영상 획득 시각이 없고, 채널 입력 만료 검사도 없다.

실제 timer 함수를 스텁 시계·메시지로 호출했을 때 입력을 갱신하지 않은 채 5초 뒤에도 같은 ranges에 새 stamp가 붙었다. 실차나 ROS 런타임 시험은 아니다. [ROS 2 Humble LaserScan 정의](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/LaserScan.msg)는 header 시각을 관측 획득 시각으로 정의한다.

추가로 `expected_update_rate: 10.0`은 [Nav2 Humble 구현](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/observation_buffer.cpp#L60)에서 초 단위 Duration으로 해석된다. 10 Hz 설정이 아니다. 타이머가 계속 재발행하면 이 설정만 바꿔도 상류 카메라/추론 중단을 판별하기 어렵다.

**조치:** 입력 프레임 식별자·획득 시각과 처리 시각을 전달·저장하고, pilot에서 입력 중단/추론 지연 시 오래된 관측의 처리를 확인한다. 모델 inference 시간, 입력 획득→scan 지연, 고유 관측 갱신 주기, Nav2 반응 시간을 분리한다. 허용 갱신 간격은 실제 처리 주기와 지연 분포로 정한다.

### 3.5. 온라인·benchmark·원고의 파이프라인이 다름

| 요소 | 온라인 detector | benchmark wrapper |
| --- | --- | --- |
| 기본 가중치 경로 | `.pt` | 옵션에 따라 OpenVINO 또는 `.pt` |
| confidence | 기본 0.15 | 0.25 |
| 클래스 선택 | 전체 OR | floor 선택 후 전체 클래스 fallback 가능 |
| closing | 7×7, 2회 | 5×5, 1회 |
| EMA/persistence | 있음 | wrapper `infer()`에는 없음 |
| 처리 주기 | 0.1초 gate | 가능한 속도로 순차 실행 |

온라인의 `inference_device` 파라미터는 선언·조회되지만 `self.model(...)`에 전달되지 않는다. OpenVINO 처리량을 실제 온라인 동작의 증거로 그대로 쓰기 어렵다. 실제 입력 tensor 크기도 실행 시 확인·보관한다.

**조치:** 동일한 perception 구현을 benchmark와 온라인에서 공유하도록 정리하거나, 적어도 위 조건을 결과마다 명시한다. 비교를 백엔드 효과와 알고리즘 효과로 나눠 보고한다.

### 3.6. 기존 주행 CSV는 성공률과 ablation을 단독 입증하지 못함

20회 추출 CSV에는 카메라·`/plan` 메시지가 모두 0개이고 action 성공/충돌/개입의 판정 열도 없다. 18행의 최종 x가 9.5 m 이상인 것은 재확인했지만, 이것만으로 18회 무개입 성공이 검증되는 것은 아니다.

또한 원고의 “20회·6,000+ RGB 프레임 SIL ablation”에 필요한 영상 입력은 이 20개 bag의 인벤토리에서 확인되지 않는다. 별도 입력이 있는지 찾아야 한다. **고정된 실제 주행 bag을 재생하는 것만으로 다른 제어 입력을 적용했을 때의 폐루프 완주율을 얻을 수는 없다.** 오프라인 ablation은 인식 오차·무검출·지연·정적 jitter 중심으로 정의하고, 구성별 완주율은 실제 재주행 또는 제어에 따라 상태가 변하는 검증된 시뮬레이션 근거가 필요하다.

`run08/run12`의 `/floor_detector=True`는 하단 ROI 신호이며 전체 인식이 정상이라는 판정이 아니다. 원인 확정에는 영상·costmap·planner 진단과 재현이 필요하다.

### 3.7. 로거와 기하 해석도 본 평가 전에 정리

- `experiments/run_experiment.py:60`은 `/run_slam/camera_pose`를 `PoseStamped`로 구독하지만 저장소의 stella publisher는 `nav_msgs/Odometry`를 발행한다(`src/stella_vslam_ros/src/stella_vslam_ros.cc:34`). 이 조합에서는 tracking 통계가 신뢰할 만한 값으로 수집되지 않는다.
- 로거의 `PASS`는 Nav2 목표 도달 판정이 아니다. 가상 scan 최소값을 실제 차체-장애물 최소 여유 거리로 해석해서도 안 된다.
- 커밋된 LUT의 최솟값은 약 2.17769 m다. 이것은 기하상 출력 하한이며 2.18 m의 안전 여유를 실제 측정했다는 뜻이 아니다.
- 141개 bin 중 현재 col→channel LUT가 연결하는 것은 127개(인덱스 7~133)다. 선언한 ±35° 전체가 같은 관측 범위인 것처럼 표현하지 말고 실제 지원 구간과 범위 밖을 도식에 표시한다.
- 기본 Nav2 launch는 map 파일과 `bringup_launch.py`를 사용한다. 새 실험에서 mapless를 주장하려면 실제 실행 노드·TF·파라미터로 해당 모드를 확인해야 한다. 현재 기본 launch만으로 과거 실험의 실행 모드를 확정할 수는 없다.

## 4. 실험과 피규어 전략

기존 계획을 활용하되 다음 증거를 완성하는 데 집중한다.

| 단계 | 실험·작업 | 우선 지표 | 피규어/표 | 시작·완료 판단 |
| --- | --- | --- | --- | --- |
| 준비 | floor 클래스·시각·실제 배포 경로·무검출 처리를 점검하고 pilot | 실제 입력/관측 주기, 누락, action/관찰 판정 연결 | 버전/백엔드/센서/설정 표 | 분석 가능한 파일이 만들어지는지 직접 개봉 확인 |
| E1 | 2개 실제 장소 × 박스/다른 형상/박스 2개 × 각 10회 = 60회 | 성공/전체, 충돌·timeout·abort·개입, 목표 수락→종료 시간, odom 기반 경로 길이 | F03 환경·배치, F04 동기화 주행 과정, F05 전체 궤적, F06 결과+CI, T01/T02 | pilot·튜닝과 본 평가 분리. 실패 포함 모든 시도 기록 |
| E2 | 기존 baseline 근거 미복구 시 2환경×3거리×3독립 재배치 = 18클립 | 독립 GT 대비 MAE/bias, 유효 검출률, 클립별 분포, 실제 처리 지연 | F08 거리별 오차+무검출+지연, T03 공정 비교표 | 6개 보정/12개 평가 split 고정, 같은 입력·표적 정의 |
| O1 | 영상이 확보된 동일 입력에서 구성요소 ablation | 비바닥 경계/거리 오차, 오탐·누락, 정지 표적 jitter, 탐지·해제 지연 | F07 동일 프레임 전후·시계열, 별도 ablation 표 | fixed replay 결과를 실제 closed-loop 성공률로 쓰지 않음 |
| D1 | 기하·버전·자료 출처 정리 | 운용 범위, LUT 지원 bin, 실제 장착과 TF | F01 pipeline, F02 blind spot/지원 영역, 실패 근거표 T04 | 이론·합성·관측 데이터를 각각 식별 |

### 피규어를 실제로 만드는 데 필요한 항목

- **환경/배치:** 장소 A/B 사진, 폭·바닥·조명·장착 측면 치수, 출발/목표와 물체 크기·좌표. 도식의 계획 좌표와 실제 측정 좌표를 분리한다.
- **동기화 주행 패널:** 한 trial의 외부 영상, RGB, floor-only mask, scan/costmap을 같은 시각으로 정렬한다. trial ID·timestamp를 표시한다. 대표 주행은 조건별 성공 시간 중앙값에 가까운 회차를 사용하고 실패 유형도 별도로 제시한다.
- **전체 궤적:** 6조건의 모든 본 평가 궤적과 장애물 외곽, outcome을 같은 축척으로 그린다. odometry 기반이며 외부 위치 GT가 아님을 표시한다.
- **결과 분포:** 조건별 성공/전체와 Wilson 95% CI, 실패 종류를 표시한다. 조건당 n=10에서 9/10의 CI는 약 59.6~98.2%, 10/10도 약 72.2~100%다. 이는 통계 예시이며 결과 예측이 아니다. 두 장소의 결과를 실내 전반으로 일반화하지 않는다.
- **거리/지연 비교:** 거리별 signed error와 무검출을 함께 표시하고, 모델 inference 시간과 획득→scan 지연을 구분한다. mask/scan 생성 파이프라인을 통일한다.

E2에서 환경별 보정을 허용하면 해당 평가는 “그 환경에서 보정 후의 성능”이다. 미지 환경 일반화를 주장하려면 보정 환경과 평가 환경도 분리해야 한다. 장소 A/B가 바닥과 조명을 동시에 바꾸는 설계라면 바닥 또는 조명만의 독립적인 효과로 해석하지 않는다.

`record_rosbag.sh nav`는 출발점으로 쓸 수 있지만 costmap과 action result를 자동으로 모두 확보하지 않는다. 현장에는 영상·scan·odom·cmd·TF·plan 외에 실제 action 결과, 충돌/개입 관찰, 런타임 파라미터와 프레임 획득/처리 시각을 함께 남긴다.

## 5. 지금 할 일과 아직 하지 않은 일

**지금 확인 완료:** Git 동기화, 리뷰/계획/주요 코드 대조, CSV 원래 집계 조건 재현, LUT 지원 범위 확인, 실제 함수에 합성 입력을 제공한 네 가지 동작 확인.

**다음 구현 작업:** floor-only 처리, benchmark 실패/무검출 및 metric 보정, acquisition timestamp/만료 처리, 로거 메시지 타입과 결과 판정, 실제 배포 파이프라인·설정 고정. 이 보고서 작성으로 해당 코드를 수정한 것은 아니다.

**실험 시작 시 필요한 확정 정보:** 실제 제출본·IJCAS 원문, 원본 bag 접근 위치와 복사본, 학습 데이터 분할/정답, 재투고 목표일·현장 날짜·장소·인원. 자료가 갖춰지면 기존 계획의 첫날 pilot으로 실제 소요 시간과 나머지 취득 일정을 확정한다.

3일은 기존 문서의 현장 목표 일정이다. 구현 보완, 데이터 보정·분석, 피규어와 원고 개정까지 끝나는 기간으로 확정하지 않는다.

## 6. 재현 명령과 제한

저장소 루트에서:

```bash
python3 paper/review/verification/20261118_post_branch_audit.py .
```

Python, NumPy, OpenCV가 필요하다. ROS·PyTorch·Ultralytics import나 모델 다운로드를 수행하지 않는다. 소스의 실제 함수를 AST로 추출해 합성 모델 출력·메시지·시계로 호출하고, 기존 CSV/LUT를 읽는다. 이 환경에는 `rclpy`, `torch`, `ultralytics`가 없어 전체 ROS 런타임·모델·실차를 검증하지 않았다.

JSON의 수치와 원본 함수 SHA256을 함께 확인한다. CSV의 물리적 GT와 당시 설정, 실제 과거 실험의 자율성·무개입 여부까지 검증한 자료는 아니다.
