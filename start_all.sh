#!/bin/bash
# ==============================================================================
# 🚀 V-LiDAR 자율주행 통합 시스템 원클릭 사전 준비 스크립트 (One-Touch Bringup)
# ==============================================================================
# 기능:
#   카메라(cam2image), 로봇 MCU, V-LiDAR(YOLO+LUT OpenVINO), Nav2 내비게이션을
#   단 하나의 명령어로 동시에 기동합니다.
#
# 사용법:
#   ./start_all.sh             # 기본 실행 (헤드리스 모드, 실험 권장)
#   ./start_all.sh --rviz      # RViz2 시각화 화면 포함 실행
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null

echo "================================================================="
echo "🚀 [사전 준비] V-LiDAR 및 Nav2 자율주행 통합 시스템 기동"
echo "================================================================="

USE_RVIZ="false"
for arg in "$@"; do
  if [[ "$arg" =~ rviz ]]; then
    USE_RVIZ="true"
  fi
done

# 1. 이전 잔여 프로세스 정리 (포트 및 시리얼 충돌 방지)
echo "🧹 [1/2] 이전 잔여 프로세스 정리 중..."
./stop_all.sh >/dev/null 2>&1 || true
sleep 1

# 2. 통합 런치 실행
echo "🤖 [2/2] 통합 시스템 기동 중..."
echo "   • 카메라 스트림      : 640x480 (/camera/image_raw)"
echo "   • 로봇 MCU 드라이버  : /dev/ttyMCU"
echo "   • V-LiDAR 인식 엔진  : OpenVINO FP16 (Intel Core Ultra 7)"
echo "   • 2D 가상 라이다     : 141ch, ±35° FOV (/scan)"
echo "   • 내비게이션 스택    : Nav2 (A* Planner + DWB Controller)"
echo "   • RViz2 시각화 GUI   : $USE_RVIZ"
echo "================================================================="
echo "💡 시스템이 준비되면 다른 터미널에서 주행 명령을 보내세요:"
echo "   1) Rosbag 녹화: ./scripts/record_rosbag.sh vlidar <세션이름>"
echo "   2) 10m 주행전송: python3 scripts/test_avoidance_goal.py -x 10.0"
echo ""
echo "👉 시스템을 종료하려면 이 터미널에서 [Ctrl + C] 를 누르세요."
echo "================================================================="

# Ctrl+C 인터럽트 시 stop_all.sh 자동 호출하여 깔끔히 정리
trap "echo -e '\n🛑 종료 신호 수신. 프로세스를 정리합니다...'; ./stop_all.sh; exit 0" SIGINT SIGTERM

ros2 launch robot_launch_package multi_node_launch.py use_rviz:=$USE_RVIZ use_sim_time:=false
