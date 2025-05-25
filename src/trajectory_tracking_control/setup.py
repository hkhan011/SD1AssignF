from glob import glob
from setuptools import find_packages, setup
import os

package_name = 'trajectory_tracking_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.txt')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_reader = trajectory_tracking_control.waypoint_reader:main',
            'pure_pursuit = trajectory_tracking_control.pure_pursuit:main',
            'pid_controller = trajectory_tracking_control.pid_controller:main',
            'cmd_mixer = trajectory_tracking_control.cmd_mixer:main',
        ],
    },
)
