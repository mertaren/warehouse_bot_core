import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get directiories
    sim_pkg_dir = get_package_share_directory('warehouse_simulation')
    nav_pkg_dir = get_package_share_directory('warehouse_navigation')
    autonomy_pkg_dir = get_package_share_directory('warehouse_autonomy')

    # 2. YAML Config path
    patrol_config = os.path.join(autonomy_pkg_dir, 'config', 'patrol_config.yaml')
    
    # Gazebo + Robot
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_dir, 'launch', 'main_simulation.launch.py')
        )
    )

    # Nav2 + AMCL + Map with 5 sec delay
    navigation_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_pkg_dir, 'launch', 'warehouse_nav.launch.py')
                ),
                launch_arguments={'use_sim_time': 'true'}.items()
            )
        ]
    )

    # Patrol
    security_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='warehouse_autonomy',
                executable='security', # entry_point
                name='security_patrol_node',
                output='screen',
                parameters=[patrol_config, # YAML file
                            {'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        simulation_launch,
        navigation_launch,
        security_node
    ])