import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
import launch_ros.actions
from ament_index_python import get_package_share_directory

def generate_launch_description():
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Package paths
    pkg_aws_world = get_package_share_directory('aws_robomaker_small_warehouse_world')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # XACRO file for Waffle Pi + OpenManipulator
    try:
        pkg_manipulation = get_package_share_directory('turtlebot3_manipulation_description')
    except Exception:
        print("ERROR: turtlebot3_manipulation_description not found!")
        print("Please run: sudo apt install ros-humble-turtlebot3-manipulation-description")
        return LaunchDescription([])

    # World launch
    aws_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_aws_world, 'launch', 'no_roof_small_warehouse.launch.py')
        ),
        launch_arguments={'gui': 'true'}.items() 
    )
    
    # Robot model
    xacro_file = os.path.join(
        pkg_manipulation,
        'urdf',
        'turtlebot3_manipulation.urdf.xacro'
    )

    # Generate URDF from XACRO
    robot_description_config = Command(['xacro ', xacro_file])

    # Robot State Publisher
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_config
        }]
    )

    # Spawn Entity
    spawn_entity_cmd = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'warehouse_bot', 
            '-x', '-5.69',
            '-y', '9.32',
            '-z', '0.05', 
            '-Y', '0.006'
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'),
            
        aws_world_cmd,
        robot_state_publisher,
        spawn_entity_cmd
    ])