import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

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

    freespace_detection = os.path.join(
        get_package_share_directory('freespace_detection'),
        'launch',
        'freespace_detection_launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz2 for visualization'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),

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
        # omo_r1_bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(omo_r1_bringup)
        ),
        # omo_r1_navigation2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(omo_r1_navigation2),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        # omo_r1_navigation2_rviz (선택적 실행)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(omo_r1_navigation2_rviz),
            condition=IfCondition(use_rviz)
        ),
        # freespace_detection
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(freespace_detection)
        ),

        # fake_lidar_with_tf 노드
        Node(
            package='fake_lidar_with_tf',
            executable='fake_lidar_with_tf',
            name='fake_lidar_node',
        ),

    ])
