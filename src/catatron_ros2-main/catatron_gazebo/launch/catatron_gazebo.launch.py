import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, AppendEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_prefix
def generate_launch_description():

    package_gazebo = 'catatron_gazebo'
    package_description = 'catatron_description'

    # Share directories
    package_share_gazebo = get_package_share_directory(package_gazebo)
    package_share_description = get_package_share_directory(package_description)

    gazebo_launch_path = PathJoinSubstitution([package_share_gazebo, 'launch','gazebo.launch.py'])
    spawn_robot_launch_path = PathJoinSubstitution([package_share_gazebo, 'launch','spawn_catatron.launch.py'])


    # *************************** NODOS ********************************************
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch_path))
    spawn_robot = IncludeLaunchDescription(PythonLaunchDescriptionSource(spawn_robot_launch_path))

    return LaunchDescription([
        spawn_robot,
        gazebo
    ])