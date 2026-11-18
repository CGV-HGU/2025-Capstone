# IJCAS AE 코멘트에 따른 실험 우선순위와 3D LiDAR 활용 판단

- 작성일: 2026-11-18
- 새 정보: 사용자가 아래 코멘트를 **IJCAS Associate Editor에게 받은 내용**이라고 직접 확인했다. 현재 2D LiDAR는 없고 3D LiDAR는 보유 가능성만 언급됐다. 모델·접근·작동 상태는 미확인이다.
- 적용: [11/16 미팅 분석](08_Week12_Meeting_Analysis_and_Updated_Experiment_Strategy_20261118.md)의 물리 LiDAR 비교안은 선택지로 유지한다. 그 문서의 40/60/120회는 해당 경로를 선택할 때의 제안이며 기본 필수 실험이 아니다.
- 기술 상태: 이 문서는 리뷰 해석과 설계 판단이다. 센서 연결·드라이버 설치·로봇 구동·새 실험은 수행하지 않았다.

## 1. 사용자 제공 IJCAS 코멘트 원문

출처는 2026-11-18 사용자 제공 텍스트다. 기존 저장소에서는 같은 본문의 단문 리뷰를 ICCAS라는 경로에 보관했지만, 현재 논의에서는 **사용자가 확인한 IJCAS AE 코멘트**를 기준으로 한다. 아래는 제공된 텍스트이며 인코딩 표기도 유지했다.

```text
Comments to author (Associate Editor)
=====================================
A practical, lightweight ideaóconverting monocular
floor-segmentation into a virtual 2D scan for Nav2 obstacle
avoidance without LiDARóthat is clearly presented and
honestly reports its limits (&#8804;0.261 m range error,
78% success rate). However, this is only a preliminary
extended abstract: validation is limited to a single static
box in one corridor, the 22% failure rate and
glossy-floor/illumination sensitivity are significant, and
comparison against a LiDAR or alternative baseline is
missing.
```

## 2. 이 코멘트가 실제로 요구하는 것

| 지적 | 필요한 증거 | 물리 LiDAR가 직접 해결하는가 |
| --- | --- | --- |
| preliminary extended abstract | 충분한 방법·학습/평가 프로토콜·원자료·반복 결과·한계 분석을 갖춘 원고 | 센서를 추가하는 것만으로 해소되지 않음 |
| single static box in one corridor | 다른 장소/경로/형상·복수 장애물에서의 반복 평가 | 비교를 더할 수 있지만 환경 확장 자체를 대체하지 못함 |
| 22% failure rate | 고정한 현재 설정에서 전체 시도·실패 유형·수정 후 재검증 | 비교 기준은 주지만 제안 시스템의 실패를 설명하거나 고치지는 않음 |
| glossy-floor/illumination sensitivity | 실제 광택·반사·조명 변화에서의 인식/회피 결과와 실패 조건 | 센서 간 차이를 볼 수 있지만 광학 조건의 통제·정답 측정이 별도로 필요 |
| LiDAR or alternative baseline 부재 | 연구 질문에 맞는 외부 비교 기법과 공정한 정량 평가 | 가능한 대응 경로 중 하나이며 유일한 경로는 아님 |

`or alternative baseline`은 물리 2D LiDAR만을 반드시 요구한 표현이 아니다. 적절한 카메라 기반 비교 기법도 대응 경로로 읽는 것이 타당하다. 다만 어떤 기법을 어떤 지표로 비교할지가 충분한지는 실제 원고·결과에 달려 있으며 수용을 보장하는 해석은 아니다.

이전 Access R3의 “같은 로봇의 물리 2D LiDAR 비교” 요구와 이번 IJCAS AE 문구는 구분한다. Access 요구 전체를 IJCAS의 필수 조건처럼 적용하지 않는다.

## 3. 물리 LiDAR 비교는 언제 의미가 있는가

의미 자체는 있다. 같은 Nav2/로봇/주행 조건에서 측거 방식이 달라질 때 성공·실패·관측 사각지대·반응 시간이 어떻게 달라지는지 기준을 제공한다. V-LiDAR가 물리 LiDAR보다 모든 지표에서 좋아야만 의미가 생기는 실험은 아니다.

다만 이번 자원 조건에서 우선순위는 다음처럼 판단한다.

| 논문의 주장 또는 목표 | 적합한 비교 | 현재 판단 |
| --- | --- | --- |
| 범용 또는 저가 평면 LiDAR 대체, 동등한 운용 신뢰성 | 대표적인 실제 센서와 같은 조건의 직접 평가 | 물리 비교 근거가 중요. 고급 3D 센서의 2D 투영만으로 저가 2D 기기의 비용·전력·동등성을 입증할 수 없음 |
| 제한된 운용 범위의 경량 monocular scan 생성·Nav2 통합 | 적절한 카메라 기반 대안, 거리/오탐/미검출/지연 및 폐루프 검증 | 이번 IJCAS 보완의 주 경로로 권장 |
| V-LiDAR 오차와 실패 조건의 보조적 해석 | 독립 거리 측정과 함께 쓰는 3D LiDAR의 2D 투영 | 장비가 준비돼 있을 때 유용한 추가 자료 |

따라서 **물리 LiDAR 비교를 무의미하다고 배제하지는 않되, 없는 2D 장비의 확보·장착을 이번 보완의 선행 조건으로 삼지 않는다.** 논문의 주장은 확보한 비교 근거와 운용 범위에 맞춘다.

## 4. 3D LiDAR를 2D scan처럼 사용하는 방법과 한계

ROS 2 Humble의 공식 `pointcloud_to_laserscan` 패키지는 `PointCloud2`를 `LaserScan`으로 변환한다. 입력을 지정 좌표계로 변환하고 높이 범위를 제한한 뒤, 각도별로 최소 평면 거리 `sqrt(x*x + y*y)`를 취하는 구현이다.

```text
3D LiDAR PointCloud2
  → 장착 TF를 반영한 좌표 변환
  → 사전에 정한 높이 구간의 점 선택
  → 각도별 최소 평면 거리로 2D scan 생성
  → 별도 scan 토픽 기록 / 필요한 경우 Nav2 입력
```

설정할 것은 목표 좌표계, 높이 범위, 각도 범위·간격, 거리 범위다. `scan_time`은 출력 필드를 채우는 값이며 해당 값을 바꾸는 것만으로 입력 센서 주기가 바뀌지 않는다. 장치 모델을 확인하기 전 특정 높이·주기·포트를 확정하지 않는다.

이 방식은 두 가지로 나눠 해석한다.

- **얇은 높이 구간/수평 ring:** 평면 스캔에 가까운 관측을 만들 수 있지만 센서의 수직 빔/스캔 패턴에 따라 점이 성기거나 빈 각도가 생길 수 있다. 수평 ring 접근 가능성도 모델별로 확인해야 한다.
- **넓은 높이 구간의 2D 투영:** 서로 다른 높이의 장애물 정보를 합친다. Nav2용 baseline은 될 수 있지만 한 수평면만 관측하는 2D LiDAR와 관측 조건이 다르다.

따라서 논문에는 **“3D LiDAR projected to 2D LaserScan”** 등으로 표기하고 원생 평면 2D LiDAR라고 쓰지 않는다. 새로운 시간 누적을 사용하면 관측 밀도·추가 지연·로봇 이동 중 왜곡도 함께 기록한다.

공통 관측 영역의 오차 비교와 각 센서 고유 범위의 주행 비교는 별도 질문이다. 제안 기법에 유리하도록 평가 영역을 사후 선택하지 않고 범위를 미리 정한다. 반사/유리 조건의 물리 LiDAR 출력을 무조건 절대 GT로 사용하지 않으며 독립 거리/물체 경계 측정을 둔다.

장비가 이미 `PointCloud2`를 안정적으로 제공한다면 먼저 작은 정적 보조 비교를 검토할 수 있다. 새 장착·드라이버·좌표 보정에 많은 작업이 필요하다면 환경 확장과 baseline 검증을 먼저 완료하는 편이 현재 리뷰 대응에 적합하다. 서로 다른 로봇으로 얻은 성공률을 센서만의 효과로 해석하지 않는다.

## 5. 권장하는 이번 주 실험의 중심

### P0. 결과를 신뢰할 수 있게 만드는 준비

[07 점검](07_Post_Branch_Audit_and_IJCAS_Priorities_20261118.md)의 floor 클래스 병합, 오래된 scan 재발행, baseline의 무검출·고정 숫자 fallback, 실제 온라인/benchmark 차이, 목표 결과와 로거 문제를 본 평가 전에 정리한다.

원고의 V-LiDAR MAE `0.192 m`와 현재 CSV/집계기의 `1.529 m`는 연결 근거가 필요하다. MiDaS/Depth Anything V2가 이미 등장한다는 사실만으로 비교가 유효해지는 것은 아니다. **독립 GT, 별도 보정 split, 동일 입력·관측 표적·후처리 정의, 실제 백엔드, 무검출 기록**을 갖춘 결과를 제시한다.

### P1. 장면 확장과 실패 검증

오석 코너 단일 장애물과 직선 복도 복수 장애물은 유효한 후보지만, 실제 공간·사각지대와 우회 가능성을 pilot에서 확인한다. 본 평가의 반복 수·속도·시작/종료·결과 분류를 미리 고정하고 충돌/timeout/abort/개입을 포함한 전체 시도를 보고한다.

기존 60회 단일 방식 계획은 선택 가능한 출발점이다. 40회 물리 baseline 비교안이 기본 요구로 바뀐 것은 아니다. 현재 자원에 맞춰 최종 조건과 횟수를 확정한다. 과거 39/50과 다른 조건의 18/20을 동일 조건 수정 효과로 단정하지 않는다.

### P1. 광택·조명 조건을 명시적으로 평가

장소 두 곳에서 성공했다는 것만으로 반사/조명 민감도 지적이 해소되지는 않는다. 같은 위치·장애물·거리에서 광원 방향이나 조명을 바꾼 짝지은 정적 클립을 확보하면 조명 영향을 더 직접적으로 볼 수 있다. 바닥 재질과 조명이 함께 바뀌면 복합 조건 평가라고 보고한다.

거리 오차뿐 아니라 오탐·미검출과 실제 장애물이 바닥으로 분류되는 오류를 포함한다. 필터 전후 비교는 같은 입력과 정답으로 수행하고, jitter 감소와 함께 작은 장애물 소실·검출 지연도 평가한다.

### P1. 적절한 시각 기반 외부 baseline

기존 MiDaS Small/Depth Anything V2를 사용한다면 임의 상대 깊이 스케일링을 정리한 후 실제 metric 보정·평가 분리로 재측정한다. 일반 MDE의 inference FPS 비교만으로 회피 시스템 간 비교를 대신하지 않는다.

센서/인식 비교로 제시할 경우 MAE·유효 검출률·오탐/미검출·처리 지연을 같은 입력·독립 GT로 평가한다. 폐루프 성능 우위를 주장하려면 baseline의 scan을 같은 Nav2에 실제로 공급하는 별도 주행이 필요하다. 자기 방법의 필터 제거 ablation만을 외부 baseline이라고 부르지 않는다.

### P2. 준비된 경우의 3D LiDAR 보조 자료

3D LiDAR의 존재·출력·장착이 확인되면 동일 장면의 RGB·3D point cloud·투영 scan을 함께 기록하는 작은 실험부터 판단한다. 이 보조 자료가 준비될 때만 범위를 확장하고, 원생 2D 센서 비교가 수행된 것처럼 기술하지 않는다.

## 6. 이에 맞는 피규어 우선순위

1. **환경·배치·운용 범위:** 장소, 장애물 형상/개수, 조명·바닥, 실제 카메라 기하와 사각지대.
2. **조건별 전체 결과:** 성공/전체와 CI, 실패 유형·시간·odom 궤적. 실패 사례에 RGB/마스크/scan/costmap/명령을 동기화한다.
3. **광택·조명 비교:** 같은 배치의 조명 변화 전후 원본/마스크/scan과 오차·오탐·미검출 분포.
4. **공정한 카메라 baseline:** 같은 입력에서의 거리 오차·관측 유효율·지연. MDE 보정과 평가 자료를 구분한다.
5. **선택적 물리 기준:** 장비가 준비되면 3D 점군과 투영 scan을 겹쳐 오차/운용 한계를 설명하는 보조 그림을 추가한다.

## 7. 기술 근거

- [ROS Perception 공식 Humble README](https://github.com/ros-perception/pointcloud_to_laserscan/blob/humble/README.md): 변환 메시지, TF·높이·각도·거리 파라미터, `scan_time` 의미.
- [공식 Humble 변환 구현](https://github.com/ros-perception/pointcloud_to_laserscan/blob/humble/src/pointcloud_to_laserscan_node.cpp): 높이 필터와 `hypot/atan2`, 각도별 최소 거리.

확인되지 않은 장비의 성능·수직 빔·최소 거리·실제 전송 주기는 여기서 가정하지 않았다.
