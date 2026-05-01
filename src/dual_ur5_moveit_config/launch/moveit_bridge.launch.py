import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config_dir = get_package_share_directory('dual_ur5_moveit_config')

    # 1. Start the Robot State Publisher to feed RViz the unified robot_description
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_config_dir, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. Start MoveIt's brain
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_config_dir, 'launch', 'move_group.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 3. Start RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_config_dir, 'launch', 'moveit_rviz.launch.py'))
    )

    # 4. Merge Gazebo's fragmented namespaces into a single channel for MoveIt
    joint_state_merger = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_merger',
        parameters=[{
            'source_list': ['/arm_1/joint_states', '/arm_2/joint_states'],
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        rsp_launch,
        move_group_launch,
        rviz_launch,
        joint_state_merger
    ])