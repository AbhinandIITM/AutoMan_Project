from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
import os
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import ExecuteProcess, TimerAction
def generate_launch_description():
    share_dir = get_package_share_directory('dual_ur5')
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    conveyor_pkg = get_package_share_directory('conveyorbelt_gazebo')
    
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
    # SCENE OBJECTS (WORK TABLE & CONVEYOR BELT)
    # ==========================================================
    table_sdf = os.path.join(share_dir, 'models', 'table', 'model.sdf')


    # ==========================================================
    # DEFAULT JOINT CONFIGURATION
    # ==========================================================
    default_joint_positions_list = [0.0, 5.417, 1.341, -0.441, 0.102, 1.732]
    
    default_joint_positions_dict_1 = {
        'arm_1_shoulder_pan_joint': 0.00,
        'arm_1_shoulder_lift_joint': 5.417,
        'arm_1_elbow_joint': 1.341,
        'arm_1_wrist_1_joint': -0.441,
        'arm_1_wrist_2_joint': 0.102,
        'arm_1_wrist_3_joint': 1.732
    }

    default_joint_positions_dict_2 = {
        'arm_2_shoulder_pan_joint': 0.00,
        'arm_2_shoulder_lift_joint': 5.417,
        'arm_2_elbow_joint': 1.341,
        'arm_2_wrist_1_joint': -0.441,
        'arm_2_wrist_2_joint': 0.102,
        'arm_2_wrist_3_joint': 1.732
    }

    # ==========================================================
    # ARM 1 DEFINITIONS
    # ==========================================================
    robot_description_content_1 = ParameterValue(
        Command(['xacro ', urdf_path, ' robot_name:=arm_1 end_effector:=vacuum']), value_type=str
    )
    rsp_1 = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='arm_1', output='screen',
        parameters=[{'robot_description': robot_description_content_1, 'use_sim_time': True, 'frame_prefix': 'arm_1_'}]
    )
    spawn_1 = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        namespace='arm_1', output='screen',
        arguments=['-entity', 'arm_1', '-topic', 'robot_description', '-robot_namespace', 'arm_1', '-x', '-0.2', '-y', '0.2', '-z', '0.72']
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
        parameters=[{
            'use_sim_time': True, 
            'robot_description': robot_description_content_1,
            'zeros': default_joint_positions_dict_1
        }],
        remappings=[('joint_states', 'joint_states_publisher')]
    )
    bridge_1 = Node(
        package='dual_ur5', executable='gui_bridge',
        namespace='arm_1', output='screen', 
        parameters=[{
            'use_sim_time': True,
            'namespace': 'arm_1',
            'initial_joint_positions': default_joint_positions_list
        }]
    )
    set_init_angle_1 = Node(
        package='dual_ur5', executable='set_joint_angles',
        namespace='arm_1', output='screen',
        parameters=[{
            "use_sim_time": True,
            "namespace": "arm_1",
            "joint_positions": default_joint_positions_list
        }]
    )

    # ==========================================================
    # ARM 2 DEFINITIONS
    # ==========================================================
    robot_description_content_2 = ParameterValue(
        Command(['xacro ', urdf_path, ' robot_name:=arm_2 end_effector:=pry_tool']), value_type=str
    )
    rsp_2 = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='arm_2', output='screen',
        parameters=[{'robot_description': robot_description_content_2, 'use_sim_time': True, 'frame_prefix': 'arm_2_'}]
    )
    spawn_2 = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        namespace='arm_2', output='screen',
        arguments=['-entity', 'arm_2', '-topic', 'robot_description', '-robot_namespace', 'arm_2', '-x', '-0.2', '-y', '0.7', '-z', '0.72']
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
        parameters=[{
            'use_sim_time': True, 
            'robot_description': robot_description_content_2,
            'zeros': default_joint_positions_dict_2
        }],
        remappings=[('joint_states', 'joint_states_publisher')]
    )
    bridge_2 = Node(
        package='dual_ur5', executable='gui_bridge',
        namespace='arm_2', output='screen', 
        parameters=[{
            'use_sim_time': True,
            'namespace': 'arm_2',
            'initial_joint_positions': default_joint_positions_list
        }]
    )
    set_init_angle_2 = Node(
        package='dual_ur5', executable='set_joint_angles',
        namespace='arm_2', output='screen',
        parameters=[{
            "use_sim_time": True,
            "namespace": "arm_2",
            "joint_positions": default_joint_positions_list
        }]
    )

    # ==========================================================
    # EVENT HANDLERS: THE SEQUENCING LOGIC
    # ==========================================================
    
    # 1. Start Arm 1 controllers AFTER Arm 1 is fully spawned
    start_controllers_1 = RegisterEventHandler(
        OnProcessExit(target_action=spawn_1, on_exit=[jsb_1, jtc_1])
    )
    # 2. Start Arm 1 GUI, Bridge, and Init Angles AFTER Arm 1 trajectory controller is active
    start_post_controllers_1 = RegisterEventHandler(
        OnProcessExit(target_action=jtc_1, on_exit=[
            gui_1, bridge_1, 
            set_init_angle_1])
    )

    # 3. CRITICAL: Do NOT even attempt to spawn Arm 2 until Arm 1 is fully spawned!
    start_spawn_2 = RegisterEventHandler(
        OnProcessExit(target_action=spawn_1, on_exit=[spawn_2])
    )

    # 4. Start Arm 2 controllers AFTER Arm 2 is fully spawned
    start_controllers_2 = RegisterEventHandler(
        OnProcessExit(target_action=spawn_2, on_exit=[jsb_2, jtc_2])
    )
    # 5. Start Arm 2 GUI, Bridge, and Init Angles AFTER Arm 2 trajectory controller is active
    start_post_controllers_2 = RegisterEventHandler(
        OnProcessExit(target_action=jtc_2, on_exit=[
            gui_2, bridge_2, 
            set_init_angle_2])
    )
    start_conveyor = TimerAction(
        period=26.0, 
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/CONVEYORPOWER', 'conveyorbelt_msgs/srv/ConveyorBeltControl', '"{power: 100.0}"'],
                shell=True,
                output='screen'
            )
        ]
    )
    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        rsp_1,             # Publish Arm 1 state
        rsp_2,             # Publish Arm 2 state
        spawn_1,           # Begin spawning Arm 1
        start_controllers_1,
        start_post_controllers_1,
        start_spawn_2,     # This waits patiently for spawn_1 to finish
        start_controllers_2,
        start_post_controllers_2,
        start_conveyor,
    ])