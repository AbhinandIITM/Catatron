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
    package_gazebo_ros = 'gazebo_ros'
    # Share directories
    package_share_gazebo = get_package_share_directory(package_gazebo)
    package_share_description = get_package_share_directory(package_description)
    package_share_gazebo_ros = get_package_share_directory(package_gazebo_ros)

    install_dir = get_package_prefix(package_description)

    # Plugins
    gazebo_plugins_name = "gazebo_plugins"
    gazebo_ros2_control_name = "gazebo_ros2_control"
    # Paths
    models_path = PathJoinSubstitution([install_dir,"share"])
    gazebo_plugins_name_path_install_dir = get_package_prefix(gazebo_plugins_name)
    gazebo_ros2_control_path_install_dir = get_package_prefix(gazebo_ros2_control_name)

    gazebo_plugins = PathJoinSubstitution([gazebo_plugins_name_path_install_dir, 'lib'])
    gazebo_plugins_ros = PathJoinSubstitution([gazebo_ros2_control_path_install_dir, 'lib'])

    
    # *************************** DECLARE ARGUMENTS ********************************************
    world_default = PathJoinSubstitution([package_share_gazebo, 'worlds','office.world'])
    arg_world = DeclareLaunchArgument(
        name = "world",
        default_value = world_default,
        description = "world for gazebo"
    )
    config_world = LaunchConfiguration('world')

    # *************************** ENVIRONMENT VARIABLE ********************************************
    gazebo_model_env = AppendEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[models_path]

    )
    gazebo_plugin_env = AppendEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=[gazebo_plugins]
    )
    gazebo_plugin_ros_env = AppendEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=[gazebo_plugins_ros]
    )
    gazebo_uri_env = AppendEnvironmentVariable(
        name="GAZEBO_MODEL/DATABASE",
        value=['http://models.gazebosim.org']
    )

    # *************************** NODOS ********************************************
    gazebo_ros_path = PathJoinSubstitution([package_share_gazebo_ros, 'launch','gazebo.launch.py'])
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_ros_path),
        launch_arguments={
            'world': config_world,
            'verbose': 'false',
            'debug': 'false',
            'pause': 'false'
        }.items())

    return LaunchDescription([
        gazebo_uri_env,
        gazebo_model_env,
        gazebo_plugin_env,
        gazebo_plugin_ros_env,
        arg_world,
        gazebo
    ])