import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_name = 'dual_ur5'
    pkg_share_dir = get_package_share_directory(pkg_name)
    
    # Extract the workspace share directory to allow Gazebo to resolve package:// URIs
    workspace_share_dir = os.path.dirname(pkg_share_dir)
    
    # Automatically set GAZEBO_MODEL_PATH
    set_gazebo_model_path = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        workspace_share_dir
    )

    urdf_file = os.path.join(
        pkg_share_dir,
        'urdf',
        'ur5e.urdf'
    )

    # ================= GAZEBO =================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'gui': 'true'}.items()
    )

    # ================= ARM 1 =================
    spawn_arm1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', urdf_file,
            '-entity', 'arm1',
            '-x', '-0.7',
            '-y', '0',
            '-z', '0.45'
        ],
        output='screen'
    )

    # ================= ARM 2 =================
    spawn_arm2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', urdf_file,
            '-entity', 'arm2',
            '-x', '0.7',
            '-y', '0',
            '-z', '0.45'
        ],
        output='screen'
    )

    # ================= RVIZ =================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        spawn_arm1,
        # spawn_arm2,
        rviz_node,
    ])