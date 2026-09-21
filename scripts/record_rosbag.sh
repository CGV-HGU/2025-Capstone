#!/bin/bash
# ==============================================================================
# V-LiDAR 및 OMO R1 주행 데이터 rosbag 자동 녹화 스크립트
# 사용법:
#   ./record_rosbag.sh [모드] [세션이름]
#
# 모드:
#   vlidar   : V-LiDAR 알고리즘 오프라인 튜닝용 (카메라 + TF + Odom)
#   nav      : 자율주행 및 회피 분석용 전체 토픽 (기본값)
#   minimal  : 최소 토픽 (스캔 + TF + Odom)
#
# 예시:
#   ./record_rosbag.sh vlidar obstacle_test_01
#   ./record_rosbag.sh nav cornering_avoidance
# ==============================================================================

MODE=${1:-"nav"}
SESSION_NAME=${2:-"session_$(date +%Y%m%d_%H%M%S)"}
BAG_BASE_DIR="$HOME/data/bags"
BAG_OUTPUT_DIR="${BAG_BASE_DIR}/${MODE}_${SESSION_NAME}"

mkdir -p "$BAG_BASE_DIR"

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null

echo "========================================================"
echo "🔴 ROS 2 Bag 녹화를 시작합니다."
echo "📌 녹화 모드: $MODE"
echo "📁 저장 경로: $BAG_OUTPUT_DIR"
echo "========================================================"

case "$MODE" in
  vlidar)
    echo "💡 [V-LiDAR 전용 모드] 카메라 원본 영상과 좌표계를 녹화합니다."
    echo "   (오프라인에서 YOLO 세그멘테이션 및 LUT 파라미터를 튜닝할 때 최적)"
    TOPICS=(
      "/camera/image_raw"
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
    echo "💡 [전체 주행 분석 모드] 카메라 영상, V-LiDAR, Nav2 계획 및 제어 토픽 전체를 녹화합니다."
    TOPICS=(
      "/camera/image_raw"
      "/scan"
      "/lidar_channel_distances"
      "/floor_detector"
      "/tf"
      "/tf_static"
      "/odom"
      "/cmd_vel"
      "/plan"
      "/local_plan"
      "/initialpose"
      "/goal_pose"
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

# ZSTD 압축 적용으로 이미지 저장 용량 절약 (file 단위 압축)
ros2 bag record \
  -o "$BAG_OUTPUT_DIR" \
  --compression-mode file \
  --compression-format zstd \
  --max-cache-size 104857600 \
  "${TOPICS[@]}"

echo ""
echo "========================================================"
echo "✅ 녹화가 완료되었습니다: $BAG_OUTPUT_DIR"
echo "💡 파일 정보 확인: ros2 bag info $BAG_OUTPUT_DIR"
echo "========================================================"
