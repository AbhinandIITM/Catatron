from setuptools import setup
import glob
import os

package_name = 'catatron_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),  # Corrected glob usage
        (os.path.join('share', package_name, 'urdf'), glob.glob('urdf/*')),
        (os.path.join('share', package_name, 'mesh_files/dae_files'), glob.glob('mesh_files/dae/*')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*')),
        (os.path.join('share', package_name, 'scripts'), glob.glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Catatron description and control in ROS 2 with Gazebo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'robot_launch = catatron_description.robot.launch:generate_launch_description',
        'controller = catatron_description.controller:main', 
    ],
    },
)
