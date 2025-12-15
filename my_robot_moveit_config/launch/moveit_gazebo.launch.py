import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import SetParameter

def generate_launch_description():
    
    # 1. PATHS
    moveit_config_pkg = get_package_share_directory('my_robot_moveit_config')
    description_pkg = get_package_share_directory('my_robot_description')

    # 2. GLOBAL PARAMETER: FORCE SIM TIME
    # This ensures MoveIt and RViz use Gazebo's clock, not the system clock.
    sim_time_param = SetParameter(name='use_sim_time', value=True)

    # 3. INCLUDE GAZEBO SIMULATION
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_pkg, 'launch', 'gazebo.launch.py')
        )
    )

    # 4. MOVEIT SETUP
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'move_group.launch.py')
        )
    )

    # 5. RVIZ SETUP
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'moveit_rviz.launch.py')
        )
    )

    return LaunchDescription([
        sim_time_param, # <--- This is the fix!
        gazebo_sim,
        move_group,
        rviz
    ])