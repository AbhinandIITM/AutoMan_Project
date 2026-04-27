from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
import os
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    share_dir = get_package_share_directory('dual_ur5')
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    workspace_share_dir = os.path.dirname(share_dir)
    
    set_gazebo_model_path = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        workspace_share_dir
    )

    urdf_path = os.path.join(share_dir, 'urdf', 'ur5e.urdf')
    world_path = os.path.join(share_dir, 'worlds', 'empty_world.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path
        }.items()
    )

    # ==========================================================
    # ARM 1 DEFINITIONS
    # ==========================================================
    robot_description_content_1 = ParameterValue(
        Command(['xacro ', urdf_path, ' robot_name:=arm_1']), value_type=str
    )
    rsp_1 = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='arm_1', output='screen',
        parameters=[{'robot_description': robot_description_content_1, 'use_sim_time': True, 'frame_prefix': 'arm_1_'}]
    )
    spawn_1 = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        namespace='arm_1', output='screen',
        arguments=['-entity', 'arm_1', '-topic', 'robot_description', '-robot_namespace', 'arm_1', '-x', '0.0', '-y', '0.0', '-z', '0.0']
    )
    jsb_1 = Node(
        package="controller_manager", executable="spawner",
        namespace="arm_1", arguments=["joint_state_broadcaster", "--controller-manager", "/arm_1/controller_manager"]
    )
    jtc_1 = Node(
        package="controller_manager", executable="spawner",
        namespace="arm_1", arguments=["joint_trajectory_controller", "-c", "/arm_1/controller_manager"]
    )
    gui_1 = Node(
        package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
        namespace='arm_1', output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description_content_1}],
        remappings=[('joint_states', 'joint_states_publisher')]
    )
    bridge_1 = Node(
        package='dual_ur5', executable='gui_bridge',
        namespace='arm_1', output='screen', parameters=[{'use_sim_time': True}]
    )

    # ==========================================================
    # ARM 2 DEFINITIONS
    # ==========================================================
    robot_description_content_2 = ParameterValue(
        Command(['xacro ', urdf_path, ' robot_name:=arm_2']), value_type=str
    )
    rsp_2 = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='arm_2', output='screen',
        parameters=[{'robot_description': robot_description_content_2, 'use_sim_time': True, 'frame_prefix': 'arm_2_'}]
    )
    spawn_2 = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        namespace='arm_2', output='screen',
        arguments=['-entity', 'arm_2', '-topic', 'robot_description', '-robot_namespace', 'arm_2', '-x', '0.0', '-y', '1.0', '-z', '0.0']
    )
    jsb_2 = Node(
        package="controller_manager", executable="spawner",
        namespace="arm_2", arguments=["joint_state_broadcaster", "--controller-manager", "/arm_2/controller_manager"]
    )
    jtc_2 = Node(
        package="controller_manager", executable="spawner",
        namespace="arm_2", arguments=["joint_trajectory_controller", "-c", "/arm_2/controller_manager"]
    )
    gui_2 = Node(
        package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
        namespace='arm_2', output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description_content_2}],
        remappings=[('joint_states', 'joint_states_publisher')]
    )
    bridge_2 = Node(
        package='dual_ur5', executable='gui_bridge',
        namespace='arm_2', output='screen', parameters=[{'use_sim_time': True}]
    )

    # ==========================================================
    # EVENT HANDLERS: THE SEQUENCING LOGIC (NO RACE CONDITIONS!)
    # ==========================================================
    
    # 1. Start Arm 1 controllers AFTER Arm 1 is fully spawned
    start_controllers_1 = RegisterEventHandler(
        OnProcessExit(target_action=spawn_1, on_exit=[jsb_1, jtc_1])
    )
    # 2. Start Arm 1 GUI AFTER Arm 1 controllers are active
    start_gui_1 = RegisterEventHandler(
        OnProcessExit(target_action=jtc_1, on_exit=[gui_1, bridge_1])
    )

    # 3. CRITICAL: Do NOT even attempt to spawn Arm 2 until Arm 1 is fully spawned!
    start_spawn_2 = RegisterEventHandler(
        OnProcessExit(target_action=spawn_1, on_exit=[spawn_2])
    )

    # 4. Start Arm 2 controllers AFTER Arm 2 is fully spawned
    start_controllers_2 = RegisterEventHandler(
        OnProcessExit(target_action=spawn_2, on_exit=[jsb_2, jtc_2])
    )
    # 5. Start Arm 2 GUI AFTER Arm 2 controllers are active
    start_gui_2 = RegisterEventHandler(
        OnProcessExit(target_action=jtc_2, on_exit=[gui_2, bridge_2])
    )

    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        rsp_1,             # Publish Arm 1 state
        rsp_2,             # Publish Arm 2 state
        spawn_1,           # Begin spawning Arm 1 (kicks off the chain)
        start_controllers_1,
        start_gui_1,
        start_spawn_2,     # This waits patiently for spawn_1 to finish!
        start_controllers_2,
        start_gui_2
    ])