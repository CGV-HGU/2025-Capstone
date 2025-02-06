from setuptools import setup, find_packages

package_name = 'freespace_detection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),  # 패키지 자동 탐색
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/freespace_detection_launch.py']),  # 런치 파일 추가
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Freespace detection package for ROS2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'floor_detector = freespace_detection.floor_detector:main',  # 실행 파일 이름 수정
        ],
    },
)

