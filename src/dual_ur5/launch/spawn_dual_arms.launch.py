import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    pkg_name = 'dual_ur5'
    pkg_share_dir = get_package_share_directory(pkg_name)

    urdf_path = os.path.join(pkg_share_dir, 'urdf', 'ur5e.urdf')
    rviz_config = os.path.join(pkg_share_dir, 'rviz', 'view.rviz')

   
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    # Fix Gazebo model path
    workspace_share_dir = os.path.dirname(pkg_share_dir)
    set_gazebo_model_path = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        workspace_share_dir
    )

    # ================= ROBOT STATE PUBLISHER =================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    joint_state_publisher = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            remappings=[
                ('/joint_states', '/joint_states_publisher')
        ]
    )

    # ================= GAZEBO =================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        )
    )

    # ================= SPAWN ROBOT (USE FILE, NOT TOPIC) =================
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=['-entity', 'ur5e', '-file', urdf_path],
        parameters=[],
    )

    delayed_controller_spawners = TimerAction(
        period=6.0,   # key fix
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
                output="screen",
            ),
        ]
    )

    # ================= RVIZ (optional) =================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        set_gazebo_model_path,
        robot_state_publisher,
        #joint_state_publisher,
        gazebo,
        spawn_robot,
        delayed_controller_spawners,
        # rviz_node,
    ])