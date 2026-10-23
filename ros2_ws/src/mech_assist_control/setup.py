from glob import glob
from setuptools import find_packages, setup


setup(
    name='mech_assist_control',
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mech_assist_control']),
        ('share/mech_assist_control', ['package.xml']),
        ('share/mech_assist_control/config', glob('config/*.yaml')),
        ('share/mech_assist_control/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Project Owners',
    maintainer_email='project-owners@example.invalid',
    description='Selectable ROS knee velocity-follow and bounded resistance strategies.',
    license='Internal Research Use - All Rights Reserved',
    entry_points={'console_scripts': [
        'knee_motor_follow_node = mech_assist_control.node:main',
        'desk_imu_follow_node = mech_assist_control.desk_node:main',
        'knee_control_node = mech_assist_control.node:main',
        'exoskeleton_control_node = mech_assist_control.robot_node:main',
    ]},
)
