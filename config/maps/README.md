# 🗺️ 주행 맵 및 보정 데이터 구성 가이드 (Map & Calibration Index)

본 디렉터리는 OMORobot R1 자율주행에 필요한 **Visual SLAM 지도**, **2D Nav2 점유 격자 지도**, **카메라 캘리브레이션** 데이터의 위치와 전환 방법을 설명합니다.

---

## 📂 1. 전체 데이터 구성 및 위치

| 구분 | 파일명 | 실제 저장 위치 | 설명 |
| :--- | :--- | :--- | :--- |
| **Visual SLAM 지도** | **`NTH4F.msg`** | `~/data/db/NTH4F.msg` | 뉴턴홀 4층 3D 특징점 맵 (92MB) - **기본 맵** |
| | **`OH3F.msg`** | `~/data/db/OH3F.msg` | 오석관 3층 3D 특징점 맵 (67MB) |
| **Nav2 2D 평면 지도** | **`NTH4F.png` / `map.yaml`** | `src/omo_r1_navigation2/map/` | 뉴턴홀 4층 2D 점유 격자 평면도 |
| | **`map.png` / `map2~5.png`** | `src/omo_r1_navigation2/map/` | 이전 스캔 단계별 2D 맵 |
| **카메라 캘리브레이션** | **`usb_webcam.yaml`** | `~/data/cam/usb_webcam.yaml` | 단안 USB 웹캠 인트린식 ($640 \times 480$) |
| | **`stereo360.yaml`** | `~/data/cam/stereo360.yaml` | 360 스테레오 카메라 파라미터 |
| **VSLAM 사전** | **`orb_vocab.fbow`** | `~/data/vocab/orb_vocab.fbow` | ORB Feature Vocabulary 바이너리 (45MB) |

---

## 🔄 2. 주행 맵 전환 방법 (예: 뉴턴홀 ➔ 오석관)

주행 대상 장소를 변경할 때는 `run_ros2_launch.sh` 파일의 맵 옵션을 한 줄만 수정하면 됩니다:

```bash
# ~/ros2_ws/run_ros2_launch.sh 내부:
# [뉴턴홀 4층 주행 시 (기본)]
--map-db-in ~/data/db/NTH4F.msg \

# [오석관 3층 주행 시]
--map-db-in ~/data/db/OH3F.msg \
```

---

## 🔨 3. 새로운 장소 맵 생성 방법
새로운 장소에서 맵을 생성할 때는 루트의 매핑 스크립트를 사용합니다:
```bash
cd ~/ros2_ws
./create_vslam_map.sh [새로운_맵이름]
# 생성된 파일은 ~/data/db/[맵이름].msg 에 자동으로 저장됩니다.
```
