#!/bin/bash

# ==============================================================================
# Visual SLAM (stella-vslam) 실시간 매핑 스크립트
# 사용법: ./create_vslam_map.sh [맵_이름]
# 예시:   ./create_vslam_map.sh NTH4F
# ==============================================================================

MAP_NAME=${1:-"NTH4F_$(date +%Y%m%d_%H%M%S)"}
OUTPUT_MAP_PATH="$HOME/data/db/${MAP_NAME}.msg"

echo "========================================================"
echo "🚀 Visual SLAM 매핑을 시작합니다!"
echo "📁 저장될 맵 파일: $OUTPUT_MAP_PATH"
echo "========================================================"

# ROS 2 환경 로드
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 1. 카메라 노드 백그라운드 실행
echo "📷 [1/2] USB 카메라 노드(cam2image)를 실행합니다..."
ros2 run image_tools cam2image --ros-args -p device_id:=0 -p width:=640 -p height:=480 &
PID_CAM=$!

# 종료 시 카메라 프로세스도 함께 정리
cleanup() {
    echo ""
    echo "🛑 매핑을 종료하고 프로세스를 정리합니다..."
    kill -9 $PID_CAM 2>/dev/null
    echo "✅ 맵 생성이 완료되었습니다: $OUTPUT_MAP_PATH"
    echo "💡 최신 맵으로 심볼릭 링크(NTH4F_latest.msg)를 갱신합니다."
    ln -sf "$OUTPUT_MAP_PATH" "$HOME/data/db/NTH4F_latest.msg"
    exit 0
}
trap cleanup SIGINT SIGTERM

sleep 1

# 2. VSLAM 매핑 모드 실행 (PangolinViewer 창 활성화)
echo "🗺️  [2/2] stella-vslam 매핑 모드를 시작합니다. (뷰어 창이 열립니다)"
echo "👉 로봇을 천천히 이동시키며 복도를 한 바퀴 돌아주세요."
echo "👉 매핑을 마치려면 이 터미널에서 Ctrl+C 를 누르세요."
echo "--------------------------------------------------------"

ros2 run stella_vslam_ros run_slam \
    -v ~/data/vocab/orb_vocab.fbow \
    -c ~/data/cam/usb_webcam.yaml \
    -o "$OUTPUT_MAP_PATH" \
    --viewer pangolin_viewer \
    --ros-args \
      -p publish_tf:=true \
      -p publish_keyframes:=true

cleanup
