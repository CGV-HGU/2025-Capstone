#!/usr/bin/env bash
set -euo pipefail

# ROS 2 환경 설정
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# launch 파일들
LAUNCH_FILES=(
  "robot_launch_package multi_node_launch.py"
)

# 띄운 프로세스 PID 저장
PIDs=()

# SIGINT/SIGTERM 발생 시 모든 자식 프로세스 그룹 종료
trap 'echo "Terminating all..."; kill 0; exit' SIGINT SIGTERM

echo "Launching ROS2 nodes..."
for launch_cmd in "${LAUNCH_FILES[@]}"; do
  # & 모드로 띄우면서 프로세스 그룹 ID로 시작하도록
  nohup ros2 launch $launch_cmd \
    &> "/tmp/${launch_cmd// /_}.log" &
  PIDs+=($!)
  echo "  - '$launch_cmd' PID=${PIDs[-1]}"
done

# 모든 백그라운드 프로세스가 종료될 때까지 대기
wait
