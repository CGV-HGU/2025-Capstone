from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_openvino = LaunchConfiguration('use_openvino', default='True')
    inference_device = LaunchConfiguration('inference_device', default='CPU')
    enable_gui = LaunchConfiguration('enable_gui', default='True')
    enable_auto_costmap_clear = LaunchConfiguration('enable_auto_costmap_clear', default='False')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_openvino',
            default_value='True',
            description='Use OpenVINO FP16 model (best_openvino_model) when available'
        ),
        DeclareLaunchArgument(
            'inference_device',
            default_value='CPU',
            description='Inference device for OpenVINO (CPU supported; GPU/NPU fall back to CPU if absent)'
        ),
        DeclareLaunchArgument(
            'enable_gui',
            default_value='True',
            description='Enable cv2.imshow GUI display'
        ),
        DeclareLaunchArgument(
            'enable_auto_costmap_clear',
            default_value='False',
            description='Allow periodic ClearEntireCostmap calls (disabled by default for blind-zone safety)'
        ),
        Node(
            package='freespace_detection',
            executable='floor_detector',
            name='floor_detector_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_openvino': use_openvino},
                {'inference_device': inference_device},
                {'enable_gui': enable_gui},
                {'enable_auto_costmap_clear': enable_auto_costmap_clear},
            ]
        )
    ])
