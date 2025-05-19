from setuptools import find_packages, setup

package_name = 'catatron_joystick'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='morg',
    maintainer_email='alaurao@uni.pe',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ramped_joystick = catatron_joystick.ramped_joystick:main",
            "keyboard_sim_joy = catatron_joystick.keyboard_sim_joy:main",
        ],
    },
)
