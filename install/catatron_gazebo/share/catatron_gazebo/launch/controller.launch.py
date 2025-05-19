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
    joystick_node = Node(
        package='joy',                  # ROS 2 package name
        executable='joy_node',          # Executable to run
        name='JOYSTICK',                # Node name
        parameters=[{'dev': '/dev/input/js0'}],  # Parameters (equivalent to joy_node/dev)
        output='screen',                # Output to console
        emulate_tty=True                # Ensures colored output in logs
    )



    return LaunchDescription([

        controller,
        ramped_controller,
        joystick_node
    ])