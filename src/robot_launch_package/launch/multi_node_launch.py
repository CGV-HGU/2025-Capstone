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

    ])
