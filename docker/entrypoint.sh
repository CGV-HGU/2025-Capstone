#!/bin/bash
set -e

# ROS 2 Humble 환경 소싱
source /opt/ros/humble/setup.bash

# 워크스페이스 빌드 결과물이 있으면 자동 소싱
if [ -f "/root/ros2_ws/install/setup.bash" ]; then
    source /root/ros2_ws/install/setup.bash
fi

exec "$@"
