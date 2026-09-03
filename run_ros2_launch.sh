#!/bin/bash

# ROS 2 환경 설정
source /opt/ros/humble/setup.bash      # ROS 2 설치 경로
source ~/ros2_ws/install/setup.bash     # 워크스페이스 환경 설정

# 첫 번째 노드를 백그라운드에서 실행 (예: multi_node_launch.py)
echo 'Launching multi_node_launch.py...'
ros2 launch robot_launch_package multi_node_launch.py &  
# 첫 번째 프로세스 ID 저장
PID1=$!

# 두 번째 노드를 새로운 터미널에서 실행 (예: run_slam)
# echo 'Launching run_slam in a new terminal...'
# gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; \
#     source ~/ros2_ws/install/setup.bash; \
#     ros2 run stella_vslam_ros run_slam \
#         -v ~/Dataset/custom_vocab_re.fbow \
#         -c ~/Dataset/usb_webcam.yaml \
#         --map-db-in ~/Dataset/20250317_OH+NTH.msg \
#         --disable-mapping \
#         --ros-args \
#           -p publish_tf:=false \
#           -p publish_keyframes:=false \
#           -p odom2d:=false; \
#     exec bash"

# version 2: GUI 터미널이 있으면 gnome-terminal, 없으면(SSH/Headless) 백그라운드 구동
if [ -n "$DISPLAY" ] && command -v gnome-terminal &> /dev/null; then
    echo 'Launching run_slam in a new terminal window...'
    gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; \
        source ~/ros2_ws/install/setup.bash; \
        ros2 run stella_vslam_ros run_slam \
            -v ~/data/vocab/orb_vocab.fbow \
            -c ~/data/cam/usb_webcam.yaml \
            --map-db-in ~/data/db/NTH4F.msg \
            --disable-mapping \
            --ros-args \
              -p publish_tf:=false \
              -p publish_keyframes:=false \
              -p odom2d:=false; \
        exec bash"
else
    echo 'Launching run_slam in background (SSH / Headless mode)...'
    ros2 run stella_vslam_ros run_slam \
        -v ~/data/vocab/orb_vocab.fbow \
        -c ~/data/cam/usb_webcam.yaml \
        --map-db-in ~/data/db/NTH4F.msg \
        --disable-mapping \
        --ros-args \
          -p publish_tf:=false \
          -p publish_keyframes:=false \
          -p odom2d:=false &
    PID2=$!
fi



# 세 번째 노드를 새로운 터미널에서 실행 (네트워크 노드)
# echo 'Launching network_control in a new terminal...'
# gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; \
#     source ~/ros2_ws/install/setup.bash; \
#     ros2 run network_control network_control_node; \
#     exec bash"

# # 4 번째 노드를 새로운 터미널에서 실행 (freespace_detection 노드)
# echo 'Launching freespace_detection in a new terminal...'
# gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; \
#     source ~/ros2_ws/install/setup.bash; \
#     ros2 launch freespace_detection freespace_detection_launch.py; \
#     exec bash"

# Ctrl+C 입력 시 구동된 모든 백그라운드 프로세스 종료
trap "echo 'Terminating processes...'; kill $PID1 $PID2 2>/dev/null; exit" SIGINT SIGTERM

# 프로세스 유지
wait
