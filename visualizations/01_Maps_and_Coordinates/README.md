# 🗺️ 01. 주행 맵 및 글로벌 좌표계 시각화 (Maps and Coordinate Systems)

본 디렉터리는 OMORobot R1 Mini 자율주행에 사용되는 **2D 점유 격자 지도(Occupancy Grid Map)**와 **Visual SLAM 키프레임 궤적**을 실측 수치(Ground Truth)에 기반하여 시각화한 자료를 보관합니다.

---

## 📊 시각화 자료 목록

### 1. `01_NTH4F_Nav2_2D_Occupancy_Grid_Map.png`
* **대상**: 뉴턴홀 4층 실주행 맵 ([`src/omo_r1_navigation2/map/NTH4F.png`](file:///home/cgv/ros2_ws/src/omo_r1_navigation2/map/NTH4F.png))
* **해상도**: $1800 \times 551$ 픽셀 (해상도 $0.05\text{m/cell}$)
* **물리적 크기**: 가로 **$90.0\text{m}$**, 세로 **$27.55\text{m}$**
* **원점 (Origin)**: `[-73.25m, -24.20m, 0.0]`
* **주요 내용**: 이미지 픽셀 좌표를 글로벌 미터($\text{m}$) 단위 좌표계로 투영하고, 글로벌 원점 $(0,0)$과 맵 원점, 그리고 실제 VSLAM 키프레임 궤적 208개를 오버레이 표시.

### 2. `02_OH3F_Visual_SLAM_Trajectory_Reconstruction.png`
* **대상**: 오석관 3층 Visual SLAM 키프레임 궤적 ([`data/db/properties/OH3F/keyframe_trajectory.txt`](file:///home/cgv/data/db/properties/OH3F/keyframe_trajectory.txt))
* **키프레임 수**: 총 **132개**
* **복도 규격**: 전진 방향 깊이($Z$축) **$11.91\text{m}$**, 횡방향 폭($X$축) **$3.62\text{m}$**
* **주요 내용**: 키프레임 번호 진행(0번 시작 ➔ 131번 종료)에 따른 이동 경로를 Top-Down 2D 산점도 및 컬러맵으로 시각화.

### 3. `03_Map_Resolution_and_Scale_Evolution.png`
* **대상**: `src/omo_r1_navigation2/map/` 내 6개 맵 파일 전체 (`NTH4F.png`, `map.png`, `map2.png`, `map3.png`, `map4.png`, `map5.png`)
* **주요 내용**: 초기 $1441 \times 485$ 복도 맵부터 최대 $5063 \times 4929$ ($253\text{m} \times 246\text{m}$) 광역 종합 지도까지의 스케일 및 커버리지 변천사 비교.
