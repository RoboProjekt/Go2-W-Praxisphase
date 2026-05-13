import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Pfade finden
    package_path = get_package_share_directory('fast_lio')
    
    config_file_name = 'go2_fusion.yaml'
    rviz_config_file = 'fastlio.rviz'

    # Argumente definieren
    config_file = LaunchConfiguration('config_file')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=config_file_name,
        description='Config file name located in config folder'
    )

    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', 
        default_value=os.path.join(package_path, 'rviz', rviz_config_file),
        description='RViz config file path'
    )

    # Der FAST_LIO Node
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            PathJoinSubstitution([package_path, 'config', config_file]),
            {'use_sim_time': True}
        ],
        sigterm_timeout='600',
        sigkill_timeout='600'
    )

    # Rviz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        declare_config_file_cmd,
        declare_rviz_config_path_cmd,
        fast_lio_node,
        rviz_node
    ])
