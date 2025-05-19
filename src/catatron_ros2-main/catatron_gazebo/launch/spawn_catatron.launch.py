import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, AppendEnvironmentVariable, ExecuteProcess,RegisterEventHandler, ExecuteProcess, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_prefix
from launch.event_handlers import OnProcessExit
def generate_launch_description():

    package_description = 'catatron_description'
    package_gazebo = 'catatron_gazebo'
    # Share directories
    package_share_gazebo = get_package_share_directory(package_gazebo)
    package_share_description = get_package_share_directory(package_description)
    
    # path meshes 
    robot_meshes_path = PathJoinSubstitution([package_share_description,'share'])
    # robot xacro
    robot_desc_path = PathJoinSubstitution([package_share_description,'urdf', 'catatron.urdf.xacro'])
    robot_desc_file = ParameterValue(Command(['xacro ',robot_desc_path]),value_type=str)

    robot_name = "catatron"

    # *************************** ENVIRONMENT VARIABLE ********************************************
    gazebo_model_env = AppendEnvironmentVariable(
       name='GAZEBO_MODEL_PATH',
        value=[robot_meshes_path]
    )
    
    # *************************** DECLARE ARGUMENTS ********************************************
    arg_use_sim_time = DeclareLaunchArgument(
        name = "use_sim_time",
        default_value = 'true',
        description = "use sim time or not"
    )
    config_use_sim_time = LaunchConfiguration('use_sim_time')
    # *************************** NODOS ********************************************
    spawn_robot = Node(
        package= 'gazebo_ros',
        executable= 'spawn_entity.py',
        name= "spawn_robot",
        arguments=['-entity', robot_name, '-x', '0.0', '-y', '0.0', '-z', '0.25', '-R', '0', '-P', '0', '-Y', '0.6',
                   '-topic','/robot_description'],
        emulate_tty='true'
    

    )
    robot_state_publisher = Node(
        name='robot_state_publisher',
        executable='robot_state_publisher',
        package='robot_state_publisher',
        parameters=[{
            'use_sim_time': config_use_sim_time,
            'robot_description': robot_desc_file,
        }],
        output='screen',
        emulate_tty='true'
    )
    controller = Node(
        name='robot_controller',
        executable='controller_quadruped',
        package='catatron_controller',
        output='screen',
        emulate_tty='true'
    )
    ramped_controller = Node(
        name='catatron_joystick',
        executable='ramped_joystick',
        package='catatron_joystick',
        output='screen',
        emulate_tty='true'
    )
     # *************************** PROCESS ********************************************
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'joint_state_broadcaster'],
        output='screen'
    )
    
    load_joint_group_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'joint_group_effort_controller'],
        output='screen'
    )
    # *************************** EVENTS ********************************************
    load_joint_state_controller_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit = [
                    LogInfo(msg='After 3 sec. controller will be loaded...'),
                    TimerAction(
                        period=3.0,
                        actions=[
                            LogInfo(msg='Try to load controllers'),
                            load_joint_state_controller
                        ]
                    )
                ]
            )
    )
    
    load_position_controller_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_group_effort_controller],
                
            )
    )

    return LaunchDescription([
        gazebo_model_env,
        arg_use_sim_time,
        robot_state_publisher,
        spawn_robot,
        load_joint_state_controller_event,
        load_position_controller_event,
        controller,
        ramped_controller
    ])