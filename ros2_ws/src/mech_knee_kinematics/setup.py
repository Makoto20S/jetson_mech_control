from glob import glob
from setuptools import find_packages, setup

package_name = 'mech_knee_kinematics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Project Owners',
    maintainer_email='project-owners@example.invalid',
    description='Receive-only dual-IMU knee flexion calibration and estimation.',
    license='Internal Research Use - All Rights Reserved',
    entry_points={'console_scripts': [
        'dual_imu_attitude_viewer = mech_knee_kinematics.attitude_dashboard:main',
        'knee_kinematics_node = mech_knee_kinematics.node:main',
        'knee_offline_loop = mech_knee_kinematics.offline_loop:main',
        'knee_motor_sim = mech_knee_kinematics.motor_sim:main',
        'ctrboard_can_dashboard = mech_knee_kinematics.can_dashboard:main',
        'shadow_assist_node = mech_knee_kinematics.shadow_assist:main',
        'gait_shadow_node = mech_knee_kinematics.gait_shadow:main',
    ]},
)
