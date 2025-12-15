import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    # 1. Specify the Name of the Package
    pkg_name = 'my_robot_description'

    # 2. Find the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'my_robot.urdf.xacro')

    # 3. Process the Xacro file
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    return LaunchDescription([
        # 4. Robot State Publisher (Broadcasts the TFs)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),

        # 5. Joint State Publisher GUI (Slider bar to move joints)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # 6. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
