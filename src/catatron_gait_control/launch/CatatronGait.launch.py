from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit, OnProcessStart
import os

def generate_launch_description():
    desc_share_dir = get_package_share_directory('catatron_description')
    gait_share_dir = get_package_share_directory('catatron_gait_control')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world',default=os.path.join(desc_share_dir,'worlds','empty_world.world'))
    urdf_file_path = os.path.join(desc_share_dir, 'urdf', 'catatron.urdf')
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    params = {'robot_description': robot_description_content}

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    joint_state_publisher_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    spawn_catatron = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_catatron',
        output='screen',
        arguments=['-entity', 'catatron', '-file', urdf_file_path],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )
    joint_angles_param = Node(
        package="catatron_description",
        executable="joint_angles_param",
        output="screen",
    )

    
    main_control = Node(
        package="catatron_gait_control",
        executable="main_control",
        output="screen",
    )

    joystick_controller = Node(
        package="catatron_gait_control",
        executable="joystick_controller",
        output="screen",
    )

    event_handler_1 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[robot_controller_spawner],
        )
    )

    event_handler_2 = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_controller_spawner,
            on_start=[main_control,joystick_controller],
        )
    )
      
  

    return LaunchDescription([
        robot_state_publisher_cmd,
        joint_state_publisher_cmd,
        gazebo_cmd,
        spawn_catatron,
        joint_angles_param,
        joint_state_broadcaster,
        event_handler_1,
        event_handler_2
    ])
