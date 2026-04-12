import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'automan_simulation'
    
    # 1. Process Xacro file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'dual_ur5e.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 2. Start Robot State Publisher (Broadcasts TF Tree)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # 3. Include standard Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # 4. Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'dual_ur5e_system'],
        output='screen'
    )

    # 5. Start RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        rviz_node
    ])