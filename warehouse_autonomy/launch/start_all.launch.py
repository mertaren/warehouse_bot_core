import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Packages
    nav_pkg = get_package_share_directory('warehouse_navigation')
    autonomy_pkg = get_package_share_directory('warehouse_autonomy')
    aws_world_pkg = get_package_share_directory('aws_robomaker_small_warehouse_world')
    manip_desc_pkg = get_package_share_directory('turtlebot3_manipulation_description')
    moveit_config_pkg = get_package_share_directory('turtlebot3_manipulation_moveit_config')

    # File Path
    xacro_file = os.path.join(manip_desc_pkg, 'urdf', 'turtlebot3_manipulation.urdf.xacro')
    world_file = os.path.join(aws_world_pkg, 'worlds', 'no_roof_small_warehouse', 'no_roof_small_warehouse.world')
    map_file = os.path.join(nav_pkg, 'maps', 'final_wh_map.yaml')
    patrol_config = os.path.join(autonomy_pkg, 'config', 'patrol_config.yaml')
    nav_params = os.path.join(nav_pkg, 'config', 'warehouse_params.yaml')

    # Robot model
    doc = xacro.process_file(xacro_file)
    robot_desc = {'robot_description': doc.toxml()}

    # CORE NODES
    
    # Gazebo Server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items()
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py'))
    )

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen', parameters=[robot_desc, {'use_sim_time': True}]
    )

    # Spawn robot
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'waffle_pi', '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )

    # CONTROLLERS
    joint_state_broadcaster = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"], output="screen")
    arm_controller = Node(package="controller_manager", executable="spawner", arguments=["arm_controller"], output="screen")
    gripper_controller = Node(package="controller_manager", executable="spawner", arguments=["gripper_controller"], output="screen")
    diff_drive_controller = Node(package="controller_manager", executable="spawner", arguments=["diff_drive_controller"], output="screen")


    # MoveIt
    moveit = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg="MoveIt Starting.."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(moveit_config_pkg, 'launch', 'move_group.launch.py')),
                launch_arguments={'use_sim_time': 'true'}.items()
            )
        ]
    )

    # Nav2
    navigation = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg="Nav2 Starting.."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav_pkg, 'launch', 'warehouse_nav.launch.py')),
                launch_arguments={'use_sim_time': 'true', 'map': map_file, 'autostart': 'true', 'params_file': nav_params}.items()
            )
        ]
    )

    # Initial Pose
    initial_pose = TimerAction(
        period=30.0,
        actions=[
            LogInfo(msg="Setting Pose.."),
            Node(
                package='warehouse_autonomy',
                executable='set_pose',
                name='initial_pose_setter',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # Security Patrol 
    patrol = TimerAction(
        period=40.0,
        actions=[
            LogInfo(msg="Patrol Starting.."),
            Node(
                package='warehouse_autonomy',
                executable='security_patrol_node',
                name='security_patrol_node',
                output='screen',
                parameters=[patrol_config, {'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        # First
        gazebo_server,
        gazebo_client,
        rsp,
        spawn,
        
        # After robot spawn
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[joint_state_broadcaster, arm_controller, gripper_controller, diff_drive_controller],
            )
        ),
        moveit,
        navigation,
        initial_pose,
        patrol
    ])