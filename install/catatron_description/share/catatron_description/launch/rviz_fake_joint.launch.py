import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    package_description = 'catatron_description'

    name_robot = 'catatron'
    # archivos
    urdf_file_name = 'catatron.urdf.xacro'
    rviz_file_name = 'rviz_fake_joints.rviz'

    # paths
    robot_desc_path = os.path.join(get_package_share_directory(package_description),'urdf', urdf_file_name)
    rviz_config_path = os.path.join(get_package_share_directory(package_description),'rviz', rviz_file_name)

    # Convierto el archivo xacro a un string
    robot_description = ParameterValue(Command(['xacro ', robot_desc_path]), value_type=str)

    # *************************** NODOS ********************************************
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            emulate_tty=True,
            parameters=[{'use_sim_time': False, 'robot_description':robot_description}],
            output='screen'
          )
    joint_state_publisher_fake = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
          )
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d' + rviz_config_path],
            output='screen'
          )
    return LaunchDescription([
        robot_state_publisher,
        #joint_state_publisher_fake,
        #rviz
    ])
