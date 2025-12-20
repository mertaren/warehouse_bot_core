import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    aws_world_pkg = get_package_share_directory('aws_robomaker_small_warehouse_world')
    manip_desc_pkg = get_package_share_directory('turtlebot3_manipulation_description')
    
    world_file = os.path.join(aws_world_pkg, 'worlds', 'no_roof_small_warehouse', 'no_roof_small_warehouse.world')
    xacro_file = os.path.join(manip_desc_pkg, 'urdf', 'turtlebot3_manipulation.urdf.xacro')

    doc = xacro.process_file(xacro_file)
    robot_desc = {'robot_description': doc.toxml()}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items()
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py'))
    )
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[robot_desc, {'use_sim_time': True}])

    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-topic', 'robot_description', '-entity', 'waffle_pi', '-x', '0.0', '-y', '0.0', '-z', '0.1'], output='screen')

    joint_state = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"], output="screen")
    arm_cont = Node(package="controller_manager", executable="spawner", arguments=["arm_controller"], output="screen")
    grip_cont = Node(package="controller_manager", executable="spawner", arguments=["gripper_controller"], output="screen")
    diff_drive_cont = Node(package="controller_manager", executable="spawner", arguments=["diff_drive_controller"], output="screen")

    return LaunchDescription([
        gazebo, gzclient, rsp, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[joint_state, arm_cont, grip_cont, diff_drive_cont]))
    ])