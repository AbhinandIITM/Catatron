from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.actions import ExecuteProcess
from launch.event_handlers import OnProcessExit
import os

def generate_launch_description():
    # Get share directories for the packages
    share_dir = get_package_share_directory('catatron_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Launch configurations (arguments you can change from the command line)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=os.path.join(
        share_dir,
        'worlds',
        'empty_world.world'
    ))
    
    urdf_file_path = os.path.join(share_dir, 'urdf', 'catatron.urdf')
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    # Parameters for robot_state_publisher
    params = {'robot_description': robot_description_content}
    
    # Robot State Publisher node
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    # Joint State Publisher node to handle joint state publishing
    joint_state_publisher_cmd = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}],
    )


    # Gazebo command to load the world (include the Gazebo launch file)
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Spawning the robot entity in Gazebo node
    spawn_catatron = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    name='spawn_catatron',
    output='screen',
    arguments=['-entity', 'catatron', '-file', urdf_file_path],
    parameters=[{'use_sim_time': use_sim_time, 'timeout': 60}],  # Set timeout as a parameter
)

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster","--controller-manager","/controller_manager"],
    )
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller","-c","/controller_manager"],
    )

    # ros2_control_node for controlling the robot
    ros2_control_node_cmd = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Return LaunchDescription with specified order of execution
    return LaunchDescription([
        RegisterEventHandler(
            event_handler = OnProcessExit(
                target_action = joint_state_broadcaster,
                on_exit=[robot_controller_spawner],
            )
        ),
        # Ensure that the robot state and joint state publisher are launched first
        robot_state_publisher_cmd,
        #joint_state_publisher_cmd,

        # Then load the Gazebo world and spawn the robot
        gazebo_cmd,
        spawn_catatron,
        joint_state_broadcaster
        

        # Finally, start the ros2_control_node for the robot controllers
        #ros2_control_node_cmd,
    ])

