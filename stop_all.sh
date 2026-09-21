#!/bin/bash
echo "=========================================="
echo " Stopping ALL ROS 2 and robot processes..."
echo "=========================================="

# 1. Graceful SIGINT (Ctrl+C equivalent) to all ROS-related processes
echo "[1/5] Sending SIGINT to ROS 2 processes..."
pkill -INT -f "ros2" 2>/dev/null
pkill -INT -f "nav2" 2>/dev/null
pkill -INT -f "rviz2" 2>/dev/null
pkill -INT -f "stella_vslam" 2>/dev/null
pkill -INT -f "floor_detector" 2>/dev/null
pkill -INT -f "fake_lidar" 2>/dev/null
pkill -INT -f "omo_r1" 2>/dev/null
pkill -INT -f "cam2image" 2>/dev/null
pkill -INT -f "component_container" 2>/dev/null
pkill -INT -f "pose_converter" 2>/dev/null
pkill -INT -f "robot_state_publisher" 2>/dev/null
pkill -INT -f "bt_navigator" 2>/dev/null
pkill -INT -f "test_avoidance_goal" 2>/dev/null
pkill -INT -f "multi_node_launch" 2>/dev/null

sleep 2

# 2. Force kill anything still alive (SIGKILL)
echo "[2/5] Force killing remaining processes..."
pkill -9 -f "ros2" 2>/dev/null
pkill -9 -f "nav2" 2>/dev/null
pkill -9 -f "rviz2" 2>/dev/null
pkill -9 -f "stella_vslam" 2>/dev/null
pkill -9 -f "floor_detector" 2>/dev/null
pkill -9 -f "fake_lidar" 2>/dev/null
pkill -9 -f "omo_r1" 2>/dev/null
pkill -9 -f "cam2image" 2>/dev/null
pkill -9 -f "component_container" 2>/dev/null
pkill -9 -f "pose_converter" 2>/dev/null
pkill -9 -f "robot_state_publisher" 2>/dev/null
pkill -9 -f "bt_navigator" 2>/dev/null
pkill -9 -f "test_avoidance_goal" 2>/dev/null
pkill -9 -f "multi_node_launch" 2>/dev/null

# 3. Stop ROS 2 daemon
echo "[3/5] Stopping ROS 2 daemon..."
source /opt/ros/humble/setup.bash 2>/dev/null
ros2 daemon stop 2>/dev/null || true
sleep 1

# 4. Kill DDS (FastDDS shared memory remnants)
echo "[4/5] Cleaning DDS shared memory..."
pkill -9 -f "fastdds" 2>/dev/null
rm -rf /dev/shm/fastrtps_* 2>/dev/null
rm -rf /tmp/.ros/ 2>/dev/null

# 5. Verify nothing remains
echo "[5/5] Checking remaining ROS processes..."
REMAINING=$(ps aux | grep -E "ros2|nav2|rviz2|omo_r1|floor_detector|fake_lidar|component_container|stella_vslam|bt_navigator" | grep -v grep | grep -v "stop_all.sh")
if [ -z "$REMAINING" ]; then
    echo ""
    echo "✅ All processes terminated cleanly!"
else
    echo ""
    echo "⚠️  Still running:"
    echo "$REMAINING" | awk '{print "  - PID:" $2, $11, $12}'
fi

echo "=========================================="
