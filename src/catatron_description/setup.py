from setuptools import setup
import glob
import os

package_name = 'catatron_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    # package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),  # Corrected glob usage
        (os.path.join('share', package_name, 'urdf'), glob.glob('urdf/*')),
        (os.path.join('share', package_name, 'mesh_files'), glob.glob('mesh_files/*')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*')),
        (os.path.join('share', package_name, 'catatron_description'), glob.glob('catatron_description/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob.glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Catatron description and control in ROS 2 with Gazebo',
    license='Apache-2.0',
    #tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'robot_launch = catatron_description.robot.launch:generate_launch_description',
        'controller = catatron_description.controller:main', 
        'inv_kin_node = catatron_description.inv_kin_node:main',
        'joint_angles_check = catatron_description.joint_angles_check:main',
        'joint_angles_param = catatron_description.joint_angles_param:main',
    ],
    },
)