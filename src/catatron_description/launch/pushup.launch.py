from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit, OnProcessStart
import os

def generate_launch_description():
    share_dir = get_package_share_directory('catatron_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Launch configurations (arguments you can change from the command line)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world',default='/home/ajoymathew07/Catatron/Catatron/src/catatron_description/worlds/empty_world.world')
    # default=os.path.join(share_dir,'worlds','empty_world.world')
    urdf_file_path = os.path.join(share_dir, 'urdf', 'catatron.urdf')
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()
    # Parameters for robot_state_publisher
    params = {'robot_description': robot_description_content}
    # Nodes
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
    
    set_stand_pose_1 = Node(
        package="catatron_description",
        executable="inv_kin_node",
        name="steering_action_client",
        output="screen",
        arguments=[
            'fr', '0.0','-0.03796', '-0.19996',
            'bl','0.0','0.03796', '-0.19996'
        ]
    )
    set_stand_pose_2 = Node(
        package="catatron_description",
        executable="inv_kin_node",
        name="steering_action_client",
        output="screen",
        arguments=[
            'br', '0.0','-0.03796', '-0.19996',
            'fl','0.0','0.03796', '-0.19996'
        ]
    )
    
    event_handler_1 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_angles_param],
        )
    )
    event_handler_2 = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_angles_param,
            on_start=[set_stand_pose_1,set_stand_pose_2],
        )
    )
    event_handler_2 = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=set_stand_pose_2,
            on_exit=[push_up_nodes[0],push_up_nodes[1]],
        )
    )
    
    push_up_1 = [
        {"args": ['fr', '0.02', '-0.05', '-0.162', 'bl', '0.02', '0.05', '-0.162']},
        {"args": ['fr', '0.02', '-0.05', '-0.122', 'bl', '0.02', '0.05', '-0.122']},
  
    ]
    push_up_2 = [
        {"args": ['br', '0.02', '-0.05', '-0.162', 'fl', '0.02', '0.05', '-0.162']},
        {"args": ['br', '0.02', '-0.05', '-0.122', 'fl', '0.02', '0.05', '-0.122']},
  
    ]
    push_up_1 *=10
    push_up_2 *=10
    
    push_up_nodes = []
    n = 10
    for i in range(n):
        push_up_nodes.append(Node(
            package="catatron_description",
            executable="inv_kin_node",
            name=f"steering_action_client_{i}",
            output="screen",
            arguments=push_up_1[i]["args"]))

        push_up_nodes.append(Node(
            package="catatron_description",
            executable="inv_kin_node",
            name=f"steering_action_client_{i + n}",
            output="screen",
            arguments=push_up_2[i]["args"]))

    # Avoid index out-of-bounds for event_handlers
    event_handlers = []
    for i in range(len(push_up_nodes) - 2):  # Adjusted to avoid bounds error
        event_handlers.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=push_up_nodes[i],
                    on_exit=[push_up_nodes[i + 2]]
                )
            )
        )
    event_handler_2_corrected = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=set_stand_pose_2,
        on_start=[push_up_nodes[0], push_up_nodes[1]],  # Now valid, push_up_nodes is defined
    )
)
    return LaunchDescription([
        robot_state_publisher_cmd,
        joint_state_publisher_cmd,
        gazebo_cmd,
        spawn_catatron,
        joint_state_broadcaster,
        robot_controller_spawner,
        event_handler_1,
        event_handler_2,
        *push_up_nodes,
        *event_handlers
        
    ])
