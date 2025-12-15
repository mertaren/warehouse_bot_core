import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    warehouse_nav_dir = get_package_share_directory('warehouse_navigation')
    
    map_file = os.path.join(warehouse_nav_dir, 'maps', 'final_wh_map.yaml')
    
    # Defauly nav2 params
    params_file = os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'params_file': params_file,
                'use_sim_time': 'true',
                'autostart': 'true'
            }.items()
        )
    ])