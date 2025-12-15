import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    # 1. SETUP: Define package name and paths
    pkg_name = 'my_robot_description'
    pkg_share = get_package_share_directory(pkg_name)
    
    # 2. PROCESS XACRO: Convert URDF.xacro to XML
    xacro_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # 3. GAZEBO SIMULATION: Include standard Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # 4. ROBOT STATE PUBLISHER: Publishes TF and robot_description topic
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 5. SPAWN ROBOT: Puts the robot into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_robot'],
        output='screen'
    )

    # 6. CONTROLLER MANAGERS: Spawners for the controllers defined in .yaml
    
    # A. Joint State Broadcaster
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # B. Arm Controller
    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    # 7. LAUNCH DESCRIPTION: Return the nodes with event handlers
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        
        # EVENT HANDLER 1: Wait for robot to spawn, then start joint state broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        
        # EVENT HANDLER 2: Wait for robot to spawn, then start arm controller
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_arm_controller],
            )
        ),
    ])