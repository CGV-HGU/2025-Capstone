#!/bin/bash
set -e

# ==============================================================================
# Intel NUC 14 Pro (cgv-omor1v2-01) 원클릭 환경 설정 및 OpenVINO 가속 스크립트
# ==============================================================================

echo "================================================================="
echo "🤖 [1/4] Intel Arc iGPU 런타임 및 OpenVINO 의존성 패키지 설치"
echo "================================================================="

sudo apt-get update -y
sudo apt-get install -y --no-install-recommends \
    intel-opencl-icd \
    intel-level-zero-gpu \
    clinfo \
    v4l-utils

# OpenVINO 파이썬 라이브러리 설치
pip install --upgrade pip
pip install openvino openvino-dev ultralytics

echo ""
echo "================================================================="
echo "⚡ [2/4] Intel Arc iGPU / NPU 가속 디바이스 확인"
echo "================================================================="

python3 -c "
import openvino as ov
core = ov.Core()
devices = core.available_devices
print(f'✅ OpenVINO 사용 가능 가속 디바이스: {devices}')
for d in devices:
    name = core.get_property(d, 'FULL_DEVICE_NAME')
    print(f'   - {d}: {name}')
"

echo ""
echo "================================================================="
echo "🚀 [3/4] YOLOv11 모델을 Intel Arc GPU용 OpenVINO FP16으로 1회 변환"
echo "================================================================="

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "$SCRIPT_DIR/src/freespace_detection/scripts/export_openvino.py"

echo ""
echo "================================================================="
echo "🔨 [4/4] ROS 2 워크스페이스 빌드 및 환경 반영"
echo "================================================================="

cd "$SCRIPT_DIR"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --allow-overriding image_tools
source install/setup.bash

echo ""
echo "================================================================="
echo "🎉 모든 환경 설정 및 가속 엔진 준비가 완료되었습니다!"
echo "👉 시스템 검증:  python3 verify_robot_env.py"
echo "👉 전체 구동:    ./run_ros2_launch.sh"
echo "================================================================="
