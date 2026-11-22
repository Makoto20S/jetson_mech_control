from pathlib import Path
import sys
r=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(r/'tools/project'))
from sync_orin import safe_path
out=safe_path(r,'.local/validation/same_cycle_windows_20261114');out.mkdir(parents=True,exist_ok=True)
# Use the exact runtime API prefix; omit only ROS SystemInterface declarations.
header=(r/'ros2_ws/src/mech_hardware_ros2_control/include/mech_hardware_ros2_control/composite_system.hpp').read_text()
header=header.split('class CompositeSystem :')[0].replace('#include "hardware_interface/system_interface.hpp"','')+'\n}\n'
stub=out/'include/mech_hardware_ros2_control';stub.mkdir(parents=True,exist_ok=True)
(stub/'composite_system.hpp').write_text(header)
import subprocess,os,json
cmd=[str(r/'.local/toolchains/zig-0.13.0/ziglang/zig.exe'),'c++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-I'+str(out/'include')]
for pkg in ['mech_bringup','mech_control_core','mech_protocol_cubemars','mech_simulation']:
 cmd+=['-I'+str(r/'ros2_ws/src'/pkg/'include')]
cmd += [str(r/'tools/project/test_same_cycle_runtime.cpp'),str(r/'ros2_ws/src/mech_bringup/src/ak30_force_runtime.cpp')]
for name in ['ak30_force_session','ak30_force_codec','ak30_force_wire','ak30_mapping']:
 cmd+=[str(r/f'ros2_ws/src/mech_protocol_cubemars/src/{name}.cpp')]
cmd+=['-o',str(out/'test.exe')]
env=dict(os.environ,ZIG_GLOBAL_CACHE_DIR=str(r/'.local/cache/zig'),ZIG_LOCAL_CACHE_DIR=str(r/'.local/cache/zig/local'))
run=subprocess.run(cmd,env=env,capture_output=True,text=True)
(out/'compile.log').write_text(run.stdout+run.stderr);print(run.stdout+run.stderr);assert run.returncode==0
run=subprocess.run([str(out/'test.exe')],capture_output=True,text=True);print(run.stdout+run.stderr)
(out/'result.txt').write_text(run.stdout+run.stderr);assert run.returncode==0
