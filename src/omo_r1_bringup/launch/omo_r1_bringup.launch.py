#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
  omo_r1_mcu_parameter = LaunchConfiguration(
    'omo_r1_mcu_parameter',
    default=os.path.join(
      get_package_share_directory('omo_r1_bringup'),
      'param/omo_r1_mcu.yaml'
    )
  )

  # omo_r1_lidar_parameter = LaunchConfiguration(
  #   'omo_r1_lidar_parameter',
  #   default=os.path.join(
  #     get_package_share_directory('omo_r1_bringup'),
  #     'param/omo_r1_lidar.yaml'
  #   )
  # )

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')

  omo_r1_description_dir = LaunchConfiguration(
    'omo_r1_description_dir',
    default=os.path.join(
      get_package_share_directory('omo_r1_description'),
      'launch'
    )
  )

  return LaunchDescription([
    DeclareLaunchArgument(
      'omo_r1_mcu_parameter',
      default_value=omo_r1_mcu_parameter
    ),

    # DeclareLaunchArgument(
    #   'omo_r1_lidar_parameter',
    #   default_value=omo_r1_lidar_parameter
    # ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/omo_r1_mcu.launch.py']),
      launch_arguments={'omo_r1_mcu_parameter': omo_r1_mcu_parameter}.items()
    ),
    
    # IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/omo_r1_lidar.launch.py']),
    #   launch_arguments={'omo_r1_lidar_parameter': omo_r1_lidar_parameter}.items()
    # ),
    
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([omo_r1_description_dir, '/robot_state_publisher.launch.py']),
      launch_arguments={'use_sim_time': use_sim_time}.items(),
    ),
  ])
