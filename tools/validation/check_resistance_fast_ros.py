"""No-device production plugin smoke: ONLY mock_components/GenericSystem."""
import argparse
from pathlib import Path
from ament_index_python.packages import get_package_prefix
import subprocess,time,signal,json
import yaml,rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from controller_manager_msgs.srv import ListControllers
from control_msgs.msg import DynamicJointState
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--controller', choices=('resistance', 'fixed-position'), default='resistance',
                    help='Select only a generic controller in the mock hardware process')
parser.add_argument('--output-dir',required=True,help='New external evidence directory (must not exist)')
args=parser.parse_args()
root=Path(args.output_dir).resolve()
root.mkdir(parents=True,exist_ok=False)
ns='/fast_resistance_shadow'
xml='''<robot name="no_device"><link name="world"/><link name="test"/>
<joint name="joint_1" type="revolute"><parent link="world"/><child link="test"/><axis xyz="0 0 1"/><limit effort="8" lower="-1" upper="1" velocity="1"/></joint>
<ros2_control name="shadow_only" type="system"><hardware><plugin>mock_components/GenericSystem</plugin></hardware><joint name="joint_1">
<command_interface name="effort"/><command_interface name="command_generation"/>
<state_interface name="position"><param name="initial_value">0</param></state_interface>
<state_interface name="velocity"><param name="initial_value">0</param></state_interface>
<state_interface name="effort"><param name="initial_value">0</param></state_interface>
<state_interface name="command_generation"><param name="initial_value">0</param></state_interface>
</joint></ros2_control></robot>'''
params={ns+'/controller_manager':{'ros__parameters':{'update_rate':500,
 'fast_gate':{'type':'mech_controllers/ResistanceTrainingController'},
 'joint_state_broadcaster':{'type':'joint_state_broadcaster/JointStateBroadcaster'}}},
 ns+'/fast_gate':{'ros__parameters':dict(joint='joint_1',minimum=0.,maximum=1.,max_slew_per_second=1000000.,
 damping=8.,torque_limit_nm=8.,buildup_nm_per_second=8.,direction_confirmation_s=.02,
 position_min_rad=-1.,position_max_rad=1.,run_limit_s=5.)}}
if args.controller == 'fixed-position':
 params[ns+'/controller_manager']['ros__parameters']['fast_gate']['type']='mech_controllers/FixedPositionImpedanceController'
 params[ns+'/fast_gate']['ros__parameters']=dict(joint='joint_1',minimum=0.,maximum=1.,
  max_slew_per_second=1000000.,stiffness_nm_per_rad=15.,damping=.5,torque_limit_nm=7.,
  position_min_rad=-1.,position_max_rad=1.,run_limit_s=5.)
(root/'parameters.yaml').write_text(yaml.safe_dump(params))
log=(root/'manager.log').open('w');manager=None;node=None
try:
 manager=subprocess.Popen([str(Path(get_package_prefix('controller_manager'))/'lib/controller_manager/ros2_control_node'),'--ros-args',
 '-r','__ns:='+ns,'--params-file',str(root/'parameters.yaml'),'-p','robot_description:='+xml],stdout=log,stderr=subprocess.STDOUT)
 for name in ('joint_state_broadcaster','fast_gate'):
  subprocess.run(['ros2','run','controller_manager','spawner',name,'--controller-manager',ns+'/controller_manager'],check=True,timeout=20,capture_output=True)
 rclpy.init();node=Node('offline_probe',namespace=ns)
 pub=node.create_publisher(Float64,'fast_gate/enable',1);generations=[]
 def receive(msg):
  if 'joint_1' in msg.joint_names:
   a=msg.interface_values[msg.joint_names.index('joint_1')]
   if 'command_generation' in a.interface_names:generations.append(a.values[a.interface_names.index('command_generation')])
 node.create_subscription(DynamicJointState,'dynamic_joint_states',receive,10)
 # Allow discovery without publishing permission.
 until=time.monotonic()+.3
 while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.002)
 initial=max(generations,default=0.)
 until=time.monotonic()+.5;next_send=0.
 while time.monotonic()<until:
  if time.monotonic()>=next_send:pub.publish(Float64(data=1.));next_send=time.monotonic()+.02
  rclpy.spin_once(node,timeout_sec=.001)
 final=max(generations,default=0.)
 assert final-initial>100,(initial,final,generations[-5:])
 until=time.monotonic()+.3
 while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.002)
 client=node.create_client(ListControllers,ns+'/controller_manager/list_controllers')
 assert client.wait_for_service(timeout_sec=2)
 future=client.call_async(ListControllers.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=2)
 state={x.name:x.state for x in future.result().controller}
 stopped = generations[-1]
 # Humble can keep the active label after update ERROR. Test the strong
 # generation/latched-output contract, not that lifecycle label.
 until=time.monotonic()+.12;next_send=0.
 while time.monotonic()<until:
  if time.monotonic()>=next_send:pub.publish(Float64(data=1.));next_send=time.monotonic()+.02
  rclpy.spin_once(node,timeout_sec=.002)
 assert generations[-1]==stopped,(stopped,generations[-1])
 result=dict(mock_only=True,controller=args.controller,initial_generation=initial,following_generation=final,
             updates_during_half_second=final-initial,generation_stopped_on_expiry=stopped, no_automatic_rearm=True, controller_label=state['fast_gate'])
 (root/'summary.json').write_text(json.dumps(result,indent=2));print(result)
finally:
 if node:node.destroy_node();rclpy.shutdown()
 if manager:
  manager.send_signal(signal.SIGINT)
  try:manager.wait(timeout=5)
  except subprocess.TimeoutExpired:manager.terminate();manager.wait(timeout=5)
 log.close()
