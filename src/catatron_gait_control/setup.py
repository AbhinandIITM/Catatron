from setuptools import find_packages, setup
import glob
import os

package_name = 'catatron_gait_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),  
        (os.path.join('share', package_name, 'catatron_gait_control'), glob.glob('catatron_gait_control/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhinand',
    maintainer_email='me23b208@smail.iitm.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_control = catatron_gait_control.CatatronMainControl:main',
            'joy = catatron_gait_control.Joy:main',
            'joystick_controller = catatron_gait_control.JoystickController:main', 


        ],
    },
)
