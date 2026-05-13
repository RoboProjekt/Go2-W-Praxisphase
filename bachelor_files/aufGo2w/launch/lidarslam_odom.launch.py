import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

    # --- KONFIGURATION ---
    lidar_topic_name = '/hesai_ros_driver/hesai/lidar_points' 
    # IMU-Topic belassen wir für die Remappings, wird aber durch 'use_imu: false' im YAML ignoriert.
    imu_topic_name = '/imu/data'

    main_param_dir = LaunchConfiguration(
        'main_param_dir',
        default=os.path.join(
            get_package_share_directory('lidarslam'),
            'param',
            'lidarslam.yaml'))
    
    rviz_param_dir = LaunchConfiguration(
        'rviz_param_dir',
        default=os.path.join(
            get_package_share_directory('lidarslam'),
            'rviz',
            'mapping.rviz'))

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Start RViz2 visualization'
    )

    # SCANMATCHER NODE (SLAM Frontend -> Liefert die Odometrie)
    scanmatcher = launch_ros.actions.Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[main_param_dir],
        remappings=[
            ('/input_cloud', lidar_topic_name),
            ('/imu', imu_topic_name)
        ],
        output='screen'
        )

    # ACHTUNG: Der 'graph_based_slam_node' wurde hier komplett entfernt, 
    # da wir keine globale Karte mehr aufbauen wollen.

    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_param_dir],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
        )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'main_param_dir',
            default_value=main_param_dir,
            description='Full path to main parameter file to load'),
        use_rviz_arg,
        
        # Nur noch die Odometrie starten:
        scanmatcher, 
        
        rviz,
    ])
