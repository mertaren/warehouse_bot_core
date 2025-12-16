import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    warehouse_nav_dir = get_package_share_directory('warehouse_navigation')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    map_file = os.path.join(warehouse_nav_dir, 'maps', 'final_wh_map.yaml')
    
    # Custom params
    params_file = os.path.join(warehouse_nav_dir, 'params', 'warehouse_params.yaml')

    return LaunchDescription([
        # Sim time args
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # Start nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,          
                'params_file': params_file, 
                'use_sim_time': use_sim_time,
                'autostart': 'true'
            }.items()
        )
    ])