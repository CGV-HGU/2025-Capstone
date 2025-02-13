from setuptools import find_packages, setup

package_name = 'network_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hyunseo Lee',
    maintainer_email='hslee@handong.ac.kr',
    description='2025-1 CGV Capstone: Robot Control System Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'network_control_node = network_control.network_control_node:main'
        ],
    },
)
