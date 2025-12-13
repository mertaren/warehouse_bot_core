import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from ament_index_python import get_package_share_directory

def generate_launch_description():
    
    # 1 package paths
    pkg_aws_world = get_package_share_directory('aws_robomaker_small_warehouse_world')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3 = get_package_share_directory('turtlebot3_gazebo')
    
    # 2 include
    aws_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_aws_world, 'launch', 'no_roof_small_warehouse.launch.py')
        )
    )
    
    # 3 robot model
    urdf_file_name = 'turtlebot3_waffle_pi.urdf'
    urdf_path = os.path.join(
        pkg_turtlebot3,
        'urdf',
        urdf_file_name
    )
    # 4 robot state publisher, tf tree
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_path]
    )
    # 5 spawn entity
    spawn_entity_cmd = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-entity', 'warehouse_bot', 
                   '-file', urdf_path,
                   '-x', '-5.634174', # set the start position
                   '-y', '9.296804',
                   '-z', '0.034221']
    )
    
    return LaunchDescription([
        aws_world_cmd,
        robot_state_publisher,
        spawn_entity_cmd
    ])