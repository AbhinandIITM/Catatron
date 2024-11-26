from setuptools import setup

package_name = 'catatron_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/catatron.urdf']),
        ('share/' + package_name + '/worlds', ['worlds/empty_world.world']),
        ('share/' + package_name + '/config', ['config/joint_controller.yaml']),
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
            'robot.launch = catatron_description.robot.launch:generate_launch_description'
            ],
    },
)
