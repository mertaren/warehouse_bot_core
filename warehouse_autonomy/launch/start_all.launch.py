import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler, LogInfo, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Path Definitions
    nav_pkg_dir = get_package_share_directory('warehouse_navigation')
    autonomy_pkg_dir = get_package_share_directory('warehouse_autonomy')
    aws_world_pkg_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    
    manip_desc_pkg = get_package_share_directory('turtlebot3_manipulation_description')
    moveit_config_pkg = get_package_share_directory('turtlebot3_manipulation_moveit_config')

    
    # Xacro file
    xacro_file = os.path.join(manip_desc_pkg, 'urdf', 'turtlebot3_manipulation.urdf.xacro')

    world_file = os.path.join(
        aws_world_pkg_dir, 
        'worlds', 
        'no_roof_small_warehouse', 
        'no_roof_small_warehouse.world'
    )

    map_file = os.path.join(nav_pkg_dir, 'maps', 'final_wh_map.yaml')

    patrol_config = os.path.join(autonomy_pkg_dir, 'config', 'patrol_config.yaml')
    
    # XACRO -> XML
    doc = xacro.process_file(xacro_file)
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}
     
    # Core Nodes

    # Gazebo Server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # Gazebo Client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        )
    )

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', 
                   '-entity', 'waffle_pi', 
                   '-x', '-5.0', '-y', '9.0', '-z', '0.1'], 
        output='screen'
    )
    
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    load_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
    )
    
    # MoveIt
    move_group_launch = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg="MoveIt"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(moveit_config_pkg, 'launch', 'move_group.launch.py')
                ),
                launch_arguments={'use_sim_time': 'true'}.items()
            )
        ]
    )

    # Nav2 
    navigation_launch = TimerAction(
        period=25.0,
        actions=[
            LogInfo(msg="Navigation"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_pkg_dir, 'launch', 'warehouse_nav.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'map': map_file,
                    'autostart': 'true',
                    'params_file': os.path.join(nav_pkg_dir, 'config', 'warehouse_params.yaml')
                }.items()
            )
        ]
    )

    # Initial pose
    set_initial_pose = TimerAction(
        period=30.0,
        actions=[
            LogInfo(msg="Pose"),
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once',
                    '/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    '{header: {frame_id: "map"}, pose: {pose: {position: {x: -5.0, y: 9.0, z: 0.0}, orientation: {x: -5.0, y: 9.0, z: 0.0, w: 1.0}}}}'
                ],
                output='screen'
            )
        ]
    )

    # Security Node)
    security_node = TimerAction(
        period=40.0,
        actions=[
            LogInfo(msg="Patrol"),
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
        gazebo_server,
        gazebo_client,
        node_robot_state_publisher,
                
        spawn_entity,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster, load_arm_controller, load_gripper_controller],
            )
        ),

        move_group_launch,
        navigation_launch,
        set_initial_pose,
        security_node
    ])