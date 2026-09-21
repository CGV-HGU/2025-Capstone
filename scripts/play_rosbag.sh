#!/bin/bash
# ==============================================================================
# ROS 2 Bag 재생 헬퍼 스크립트
# 사용법:
#   ./play_rosbag.sh [배그_경로] [옵션: 재생배속]
# 예시:
#   ./play_rosbag.sh ~/data/bags/vlidar_obstacle_test_01 1.0
# ==============================================================================

BAG_PATH=$1
RATE=${2:-"1.0"}

if [ -z "$BAG_PATH" ]; then
  echo "사용법: $0 <rosbag_디렉토리_경로> [배속(기본 1.0)]"
  echo "최근 녹화된 배그 목록:"
  ls -dt $HOME/data/bags/* 2>/dev/null | head -n 5
  exit 1
fi

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null

echo "========================================================"
echo "▶️  ROS 2 Bag 재생을 시작합니다."
echo "📁 대상 파일: $BAG_PATH"
echo "⏩ 재생 배속: ${RATE}x"
echo "⏰ 시뮬레이션 클록(/clock)을 함께 발행합니다."
echo "========================================================"
echo "💡 팁: 재생 중 일시정지는 Spacebar 키를 누르세요."
echo "--------------------------------------------------------"

ros2 bag play "$BAG_PATH" --clock --rate "$RATE"
