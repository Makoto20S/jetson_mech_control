"""Load calibrated selected servo joints; leave the trajectory controller inactive."""
from pathlib import Path
import tempfile
import yaml
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def read_description(path):
    return Path(path).read_text(encoding='utf-8')


def controller_config(description):
    names=[joint.attrib['name'] for joint in ET.fromstring(description).findall('./ros2_control/joint')]
    if names not in (['motor104_joint','motor105_joint'],['motor105_joint']):
        raise ValueError('Expected calibrated 104/105 pair or selected motor105 URDF')
    controllers=yaml.safe_load(Path(__file__).with_name('servo_controllers.yaml').read_text())
    controllers['servo_trajectory_controller']['ros__parameters']['joints']=names
    return controllers


def mark_ready(completed, name, returncode, path):
    if returncode != 0:
        raise RuntimeError('Controller loading failed: '+name)
    completed.add(name)
    if completed == {'state', 'trajectory'} and path:
        Path(path).write_text('controllers loaded; trajectory inactive\n')


def setup(context):
    description = read_description(LaunchConfiguration('urdf').perform(context))
    namespace = LaunchConfiguration('namespace', default='').perform(context).strip('/')
    ready_file = LaunchConfiguration('ready_file', default='').perform(context)
    controllers = controller_config(description)
    prefix = '/' + namespace if namespace else ''
    scoped = {prefix + '/' + name: values for name, values in controllers.items()}
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump(scoped, stream)
        params_path = stream.name

    def cleanup(context):
        Path(params_path).unlink(missing_ok=True)
        if ready_file:
            Path(ready_file).unlink(missing_ok=True)
        return []

    manager = prefix + '/controller_manager'
    state = Node(package='controller_manager', executable='spawner', namespace=namespace,
                 arguments=['joint_state_broadcaster', '-c', manager], output='screen')
    trajectory = Node(package='controller_manager', executable='spawner', namespace=namespace,
                      arguments=['servo_trajectory_controller', '--inactive', '-c', manager],
                      output='screen')
    completed = set()

    def finished(name):
        def callback(event, context):
            mark_ready(completed, name, event.returncode, ready_file)
            return []
        return callback

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        RegisterEventHandler(OnProcessExit(target_action=state, on_exit=finished('state'))),
        RegisterEventHandler(OnProcessExit(target_action=trajectory, on_exit=finished('trajectory'))),
        Node(package='controller_manager', executable='ros2_control_node', namespace=namespace,
             parameters=[ParameterFile(params_path),
                         {'robot_description': ParameterValue(description, value_type=str)}],
             output='screen'),
        state,
        trajectory,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf', description='Generated calibrated servo URDF'),
        DeclareLaunchArgument('namespace', default_value='', description='Isolated controller graph'),
        DeclareLaunchArgument('ready_file', default_value='', description='Optional startup completion marker'),
        OpaqueFunction(function=setup),
    ])
