import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    omo_r1_bringup = os.path.join(
        get_package_share_directory('omo_r1_bringup'),
        'launch',
        'omo_r1_bringup.launch.py'
    )

    omo_r1_navigation2 = os.path.join(
        get_package_share_directory('omo_r1_navigation2'),
        'launch',
        'navigation2.launch.py'
    )

    omo_r1_navigation2_rviz = os.path.join(
        get_package_share_directory('omo_r1_navigation2'),
        'launch',
        'navigation2_rviz.launch.py'
    )

    return LaunchDescription([
        # cam2image 노드
        Node(
            package='image_tools',
            executable='cam2image',
            name='cam2image',
            output='screen',
            parameters=[
                {'device_id': 0}, # 여기서 카메라 아이디 바꾸면 됨
                {'width': 640},
                {'height': 480}
            ],
        ),
        # pose_converter
        Node(
            package='pose_converter',
            executable='pose_converter_node',
            name='pose_converter_node'
        ),
        #omo_r1_bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(omo_r1_bringup)
        ),
        #omo_r1_navigation2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(omo_r1_navigation2),
        ),
        #omo_r1_navigation2_rviz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(omo_r1_navigation2_rviz)
        ),

        # freespace_detection 패키지의 노드 실행
        # Node(
        #     package='freespace_detection',  # 패키지 이름
        #     executable='floor_detection',    # 실행 파일 이름 (freespace_detection의 노드 실행 파일)
        #     name='freespace_detector',      # 런치 시 사용할 노드 이름
        #     output='screen'                 # 출력 형식
        # ),

    ])
