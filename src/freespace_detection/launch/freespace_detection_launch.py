from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='freespace_detection',  # 패키지 이름
            executable='floor_detector',   # 실행 파일 이름 (setup.py에 정의한 entry point)
            name='floor_detector_node',    # 노드 이름
            output='screen',               # 출력 방식 ('log' 또는 'screen')
            emulate_tty=True,              # 출력이 터미널에 깔끔하게 표시되도록 설정
            parameters=[
                # 필요한 경우 여기에 파라미터 파일이나 값을 추가할 수 있습니다.
            ]
        )
    ])
