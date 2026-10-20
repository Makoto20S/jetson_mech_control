from setuptools import setup


setup(
    name='mech_assist_control',
    version='0.1.0',
    packages=['mech_assist_control'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mech_assist_control']),
        ('share/mech_assist_control', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Project Owners',
    maintainer_email='project-owners@example.invalid',
    description='Selectable ROS knee velocity-follow and resistance-preview modes.',
    license='Internal Research Use - All Rights Reserved',
    entry_points={'console_scripts': [
        'knee_motor_follow_node = mech_assist_control.node:main',
        'knee_control_node = mech_assist_control.node:main',
    ]},
)
