from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    inference_device = LaunchConfiguration('inference_device', default='GPU')
    enable_gui = LaunchConfiguration('enable_gui', default='True')

    return LaunchDescription([
        DeclareLaunchArgument(
            'inference_device',
            default_value='GPU',
            description='Inference device for OpenVINO (GPU: Intel Arc, NPU: Intel AI Boost, CPU: CPU)'
        ),
        DeclareLaunchArgument(
            'enable_gui',
            default_value='True',
            description='Enable cv2.imshow GUI display'
        ),
        Node(
            package='freespace_detection',
            executable='floor_detector',
            name='floor_detector_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'inference_device': inference_device},
                {'enable_gui': enable_gui}
            ]
        )
    ])
