import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'catatron_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*yaml')))
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
            "controller_quadruped = catatron_controller.controller_quadruped:main",
            "test_caminata = catatron_controller.test_caminata:main",
            "controller_champ = catatron_controller.controller_champ:main"
        ],
    },
)
