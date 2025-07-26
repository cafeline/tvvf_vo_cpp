#!/usr/bin/env python3
"""
TVVF-VO C++版 Launch file
Time-Varying Vector Field と Velocity Obstacles を統合したナビゲーションシステム (C++実装)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # パッケージディレクトリを取得
    pkg_dir = get_package_share_directory('tvvf_vo_c')
    
    # パラメータファイルのパス
    params_file = os.path.join(pkg_dir, 'config', 'tvvf_vo_c_params.yaml')
    
    # Launch引数を定義
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file to use for the TVVF-VO node'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )
    
    def launch_setup(context, *args, **kwargs):
        # パラメータファイルのパスを取得
        params_file = LaunchConfiguration('params_file').perform(context)
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
        log_level = LaunchConfiguration('log_level').perform(context)
        
        # TVVF-VO C++ノード
        tvvf_vo_node = Node(
            package='tvvf_vo_c',
            executable='tvvf_vo_c_node',
            name='tvvf_vo_c_node',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time == 'true'}
            ],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[
                # 必要に応じてトピック名のリマッピングを追加
                # ('/scan', '/your_scan_topic'),
                # ('/map', '/your_map_topic'),
            ]
        )
        
        return [tvvf_vo_node]
    
    # LaunchDescriptionを作成
    ld = LaunchDescription()
    
    # Launch引数を追加
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_log_level_cmd)
    
    # ノード起動設定を追加
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld