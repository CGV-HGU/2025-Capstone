#!/bin/bash
# ==============================================================================
# V-LiDAR 및 OMO R1 주행 데이터 rosbag 자동 녹화 스크립트
# 사용법:
#   ./record_rosbag.sh [모드] [세션이름]
#
# 모드:
#   nav      : 실차 본평가/파일럿용 전체 토픽 (카메라+scan+odom+plan+costmap+Nav2 action) [기본값]
#   vlidar   : 정적 클립(D1/D2) 및 오프라인 인식/LUT 튜닝용 (카메라 + scan + TF + odom)
#   minimal  : 경량 모드 (영상 제외, scan + TF + odom + cmd_vel)
#
# 예시:
#   ./record_rosbag.sh nav pilot_placeA_S1_01
#   ./record_rosbag.sh vlidar static_d1_2p5m_deg0_rep1
# ==============================================================================

MODE=${1:-"nav"}
SESSION_NAME=${2:-"session_$(date +%Y%m%d_%H%M%S)"}
BAG_BASE_DIR="$HOME/data/bags"
BAG_OUTPUT_DIR="${BAG_BASE_DIR}/${MODE}_${SESSION_NAME}"

mkdir -p "$BAG_BASE_DIR"

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null

# Ctrl+C 종료 시 터미널 먹통(echo 꺼짐) 방지 자동 복구 핸들러
cleanup() {
    stty sane 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "========================================================"
echo "🔴 ROS 2 Bag 녹화를 시작합니다."
echo "📌 녹화 모드: $MODE"
echo "📁 저장 경로: $BAG_OUTPUT_DIR"
echo "========================================================"

case "$MODE" in
  vlidar)
    echo "💡 [V-LiDAR / 정적 D1·D2 모드] 카메라 원본 영상과 가상 스캔·좌표계를 녹화합니다."
    TOPICS=(
      "/camera/image_raw"
      "/camera/camera_info"
      "/tf"
      "/tf_static"
      "/odom"
      "/lidar_channel_distances"
      "/floor_detector"
      "/scan"
    )
    ;;

  minimal)
    echo "💡 [경량 모드] 영상 제외, 가상 라이다 및 주행 데이터만 녹화합니다."
    TOPICS=(
      "/scan"
      "/lidar_channel_distances"
      "/floor_detector"
      "/tf"
      "/tf_static"
      "/odom"
      "/cmd_vel"
    )
    ;;

  nav|*)
    echo "💡 [전체 실차 분석 모드] 영상 + V-LiDAR + Nav2 경로 + Costmap + Action 상태를 모두 녹화합니다."
    TOPICS=(
      "/camera/image_raw"
      "/camera/camera_info"
      "/scan"
      "/lidar_channel_distances"
      "/floor_detector"
      "/tf"
      "/tf_static"
      "/odom"
      "/cmd_vel"
      "/plan"
      "/local_plan"
      "/local_costmap/costmap"
      "/local_costmap/costmap_updates"
      "/global_costmap/costmap"
      "/initialpose"
      "/goal_pose"
      "/navigate_to_pose/_action/status"
      "/navigate_to_pose/_action/feedback"
      "/rosout"
    )
    ;;
esac

echo "📋 녹화 대상 토픽:"
for t in "${TOPICS[@]}"; do
  echo "   - $t"
done
echo "--------------------------------------------------------"
echo "👉 녹화를 중지하려면 Ctrl+C 를 누르세요."
echo "--------------------------------------------------------"

# ZSTD 파일 단위 압축 (image_tools cam2image 원본 토픽 용량 절감)
ros2 bag record \
  -o "$BAG_OUTPUT_DIR" \
  --compression-mode file \
  --compression-format zstd \
  --max-cache-size 104857600 \
  "${TOPICS[@]}"

stty sane 2>/dev/null

echo ""
echo "========================================================"
echo "✅ 녹화가 완료되었습니다: $BAG_OUTPUT_DIR"
if command -v ros2 >/dev/null 2>&1 && [ -d "$BAG_OUTPUT_DIR" ]; then
  echo "📊 녹화 토픽 메시지 수 검증 (ros2 bag info):"
  ros2 bag info "$BAG_OUTPUT_DIR" || true
fi
echo "========================================================"
