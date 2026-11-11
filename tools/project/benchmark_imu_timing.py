"""Bounded receive-only timing experiment. Run on Orin after Windows preflight.

No motor node or command publisher is created. All CSV buffers are bounded;
the optional legacy observer deliberately reproduces synchronous JSON logging.
"""
import argparse
from array import array
import csv
from datetime import datetime, timezone
import gc
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

FIELDS = ('event,index,source_stamp_ns,entry_ns,end_ns,state_stamp_ns,'
          'imu1_stamp_ns,imu2_stamp_ns,imu1_valid,imu2_valid,'
          'imu1_receive_age_ms,imu2_receive_age_ms').split(',')


class Buffer:
    def __init__(self, capacity):
        if type(capacity) is not int or not 1 <= capacity <= 2000000:
            raise ValueError('capacity must be 1..2000000')
        self.capacity, self.size, self.dropped = capacity, 0, 0
        self.data = array('q', [0]) * (capacity * len(FIELDS))

    def add(self, values):
        if len(values) != len(FIELDS):
            raise ValueError('wrong row width')
        if self.size == self.capacity:
            self.dropped += 1
            return
        offset = self.size * len(FIELDS)
        self.size += 1
        i = 0
        while i < len(FIELDS):
            self.data[offset+i] = int(values[i])
            i += 1

    def dump(self, path):
        with Path(path).open('w', newline='') as f:
            w = csv.writer(f); w.writerow(FIELDS)
            for n in range(self.size):
                row = list(self.data[n*len(FIELDS):(n+1)*len(FIELDS)])
                row[0] = ('imu', 'state', 'gc')[row[0]]
                row[10] /= 1e6; row[11] /= 1e6
                w.writerow(row)


def stamp_ns(stamp):
    return stamp.sec*1000000000 + stamp.nanosec


def age_ns(value):
    return round(value*1e6) if math.isfinite(value) else -1


def read_text(path):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return None


def process_stats(pid):
    try:
        raw = Path(f'/proc/{pid}/stat').read_text()
        fields = raw[raw.rfind(')')+2:].split()
        return {'pid': pid, 'minor_faults': int(fields[7]), 'major_faults': int(fields[9]),
                'utime_ticks': int(fields[11]), 'stime_ticks': int(fields[12]),
                'rss_pages': int(fields[21]), 'threads': int(fields[17])}
    except (OSError, IndexError, ValueError):
        return {'pid': pid, 'absent': True}


def resource_worker(args):
    mapping = json.loads(args.pids)
    with args.output.open('w') as f:
        deadline = time.monotonic()+args.duration
        while time.monotonic() < deadline:
            row = {'mono_ns': time.monotonic_ns(), 'processes': {k: process_stats(v) for k,v in mapping.items()},
                   'loadavg': read_text('/proc/loadavg'),
                   'frequency_khz': {p.parent.parent.name: read_text(p) for p in Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq/scaling_cur_freq')},
                   'temperature_mC': {read_text(p/'type') or p.name: read_text(p/'temp') for p in Path('/sys/class/thermal').glob('thermal_zone*')}}
            f.write(json.dumps(row)+'\n'); f.flush()
            time.sleep(1)
    return 0


def model_worker(args):
    sys.path.insert(0, str(args.model_dir))
    from ref01_dynamics_api import REF01Dynamics
    import numpy as np
    api = REF01Dynamics()
    step = 5000000
    q = np.array([0.1, 0.3]); dq = np.array([0.1, -0.1]); ddq = np.array([0.2, 0.1])
    for _ in range(64):
        api.inverse_dynamics(q,dq,ddq)
    start = time.monotonic_ns()
    release = start
    deadline = start+int(args.duration*1e9)
    count = misses = skipped = 0
    latencies = array('q')
    while release < deadline:
        now = time.monotonic_ns()
        if now < release:
            time.sleep((release-now)/1e9)
        api.inverse_dynamics(q,dq,ddq)
        finish = time.monotonic_ns()
        latencies.append(finish-release); count += 1
        misses += finish-release > step
        release += step
        if finish >= release:
            skip = (finish-release)//step+1
            skipped += skip; release += skip*step
    args.output.write_text(json.dumps({'kind':'independent_ref01_inverse_dynamics_200Hz',
        'source':'synthetic fixed valid model state, no hardware inputs or outputs',
        'count':count,'misses':misses,'skipped':skipped,'start_ns':start,'end_ns':time.monotonic_ns(),
        'max_release_to_finish_ms':max(latencies,default=0)/1e6},indent=2)+'\n')
    return 0


def stop_child(proc, grace=15):
    def send(sig):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            pass  # Child exited between poll and signal; still reap it below.
    if proc.poll() is None:
        send(signal.SIGINT)
        try:
            proc.wait(timeout=grace)
        except subprocess.TimeoutExpired:
            send(signal.SIGTERM)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                send(signal.SIGKILL); proc.wait(timeout=5)
    return proc.returncode


def run(args):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from std_msgs.msg import UInt32
    from diagnostic_msgs.msg import DiagnosticArray
    from mech_exo_state.msg import RobotState
    from check_imu_device import check
    root = Path(__file__).resolve().parents[2]
    out = args.output.resolve()
    allowed = (root/'.local/validation').resolve()
    if not out.is_relative_to(allowed) or out == allowed:
        raise ValueError('output must be a new subdirectory of repository .local/validation')
    out.mkdir(parents=True, exist_ok=False)
    device = check(args.device, args.motor_config)
    # Dedicated domain prevents commands from unrelated ROS consumers.
    os.environ['ROS_DOMAIN_ID'] = '179'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS_LOG_DIR'] = str(out/'ros_logs')
    capacity = int((args.warmup+args.duration+20)*450+20000)
    buffer = Buffer(capacity)
    result = {'schema_version':1,'started_at':datetime.now(timezone.utc).isoformat(),
              'duration_s':args.duration,'warmup_s':args.warmup,'trace_enabled':args.trace,
              'load_kind':args.load,'observer_mode':args.observer_mode,'device_check':device,
              'environment':{'kernel':os.uname().release,'python':sys.version,
                             'clock':'CLOCK_MONOTONIC, same Orin host only',
                             'clock_ticks':os.sysconf('SC_CLK_TCK'),'page_size':os.sysconf('SC_PAGE_SIZE'),
                             'ros':{k:os.getenv(k) for k in ('ROS_DISTRO','ROS_DOMAIN_ID','RMW_IMPLEMENTATION')},
                             'cpu_governor':read_text('/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor'),
                             'sync_state':json.loads((root/'.local/sync/state.json').read_text())['stamp']},
              'completed':False}
    children = {}; logs = []; gc_starts = {}; diagnostics = []; device_times = []
    imu_counts = [0,0]; state_count = 0
    legacy_records = []
    legacy = (out/'legacy_messages.jsonl').open('w') if args.observer_mode == 'legacy' else None

    def record_legacy(topic, value):
        if legacy is not None:
            item = {'monotonic_seconds':time.monotonic(),'topic':topic,**value}
            legacy_records.append(item)
            legacy.write(json.dumps(item,allow_nan=False)+'\n')

    def gc_event(phase, info):
        generation = info['generation']
        if phase == 'start':
            gc_starts[generation] = time.monotonic_ns()
        else:
            end = time.monotonic_ns(); begin = gc_starts.pop(generation,end)
            buffer.add((2,generation,0,begin,end,0,0,0,0,0,0,0))

    def imu(index,msg):
        begin = time.monotonic_ns(); source = stamp_ns(msg.header.stamp)
        imu_counts[index-1] += 1
        if legacy is not None:
            record_legacy(f'imu/imu{index}/data', {'frame_id':msg.header.frame_id,
                'stamp':[msg.header.stamp.sec,msg.header.stamp.nanosec],
                'quaternion_xyzw':[msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w],
                'angular_velocity':[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z],
                'acceleration':[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z],
                'covariance_first':[msg.orientation_covariance[0],msg.angular_velocity_covariance[0],msg.linear_acceleration_covariance[0]]})
        buffer.add((0,index,source,begin,time.monotonic_ns(),0,0,0,0,0,0,0))

    def state(msg):
        nonlocal state_count
        begin = time.monotonic_ns(); state_count += 1
        values = (1,0,0,begin,0,stamp_ns(msg.header.stamp),
            stamp_ns(msg.imus[0].header.stamp),stamp_ns(msg.imus[1].header.stamp),
            int(msg.sources[1].valid),int(msg.sources[2].valid),
            age_ns(msg.sources[1].receive_age_ms),age_ns(msg.sources[2].receive_age_ms))
        buffer.add(values[:4]+(time.monotonic_ns(),)+values[5:])

    def diagnostic(msg):
        for item in msg.status:
            row = {'mono_ns':time.monotonic_ns(),'values':{kv.key:kv.value for kv in item.values}}
            diagnostics.append(row)
            record_legacy('ctrboard/diagnostics',row)

    def device_time(msg):
        device_times.append((time.monotonic_ns(),msg.data))
        record_legacy('ctrboard/timestamp_ms',{'timestamp_ms':msg.data})

    def spawn(name, command, env=None):
        log = (out/(name+'.log')).open('w'); logs.append(log)
        proc = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,
                                start_new_session=True,env=env)
        children[name] = proc
        return proc

    node = None
    rclpy.init()
    gc.callbacks.append(gc_event)
    try:
        node = rclpy.create_node('imu_timing_observer')
        subs = [node.create_subscription(Imu,f'/imu/imu{i}/data',lambda m,i=i:imu(i,m),qos_profile_sensor_data) for i in (1,2)]
        subs += [node.create_subscription(RobotState,'/ph_exo/state',state,qos_profile_sensor_data),
                 node.create_subscription(DiagnosticArray,'/ctrboard/diagnostics',diagnostic,10),
                 node.create_subscription(UInt32,'/ctrboard/timestamp_ms',device_time,qos_profile_sensor_data)]
        state_cmd = [str(root/'.local/ros/state/install/mech_exo_state/lib/mech_exo_state/state_node.py')]
        if args.trace:
            state_cmd += ['--ros-args','-p',f'timing_trace_path:={out / "state.csv"}',
                          '-p',f'timing_capacity_rows:={capacity}']
        spawn('state',state_cmd)
        ready_deadline = time.monotonic()+8
        while state_count < 5 and time.monotonic() < ready_deadline:
            rclpy.spin_once(node,timeout_sec=.01)
        if state_count < 5 or children['state'].poll() is not None:
            raise RuntimeError('State not ready; sensor bridge not started')
        bridge_cmd = [str(root/'.local/ros/imu/install/mech_ctrboard_bridge/lib/mech_ctrboard_bridge/ctrboard_bridge_node'),
                      '--ros-args','-p','transport:=usb_cdc','-p',f'device_path:={args.device}','-p','logical_bus:=5']
        if args.trace:
            bridge_cmd += ['-p',f'timing_trace_path:={out / "bridge"}',
                           '-p',f'timing_trace_capacity:={int((args.warmup+args.duration+20)*120)}']
        spawn('bridge',bridge_cmd)
        sensor_deadline = time.monotonic()+8
        while min(imu_counts) < 20 and time.monotonic() < sensor_deadline:
            if children['bridge'].poll() is not None:
                raise RuntimeError('Bridge exited at startup')
            rclpy.spin_once(node,timeout_sec=.01)
        if min(imu_counts) < 20:
            raise RuntimeError('Both IMU streams did not become ready')
        duration_total = args.warmup+args.duration
        if args.load == 'model':
            spawn('model',[str(args.model_python),'-B',str(Path(__file__).resolve()),'model-worker',
                '--model-dir',str(root/'mujoco/ref01'),'--duration',str(duration_total),
                '--output',str(out/'model_result.json')])
        pids = {'observer':os.getpid(),**{k:v.pid for k,v in children.items()}}
        spawn('resources',[sys.executable,'-B',str(Path(__file__).resolve()),'resources',
                           '--pids',json.dumps(pids),'--duration',str(duration_total+1),
                           '--output',str(out/'resources.jsonl')])
        result['start_mono_ns'] = time.monotonic_ns()
        result['warmup_end_ns'] = result['start_mono_ns']+int(args.warmup*1e9)
        result['measurement_end_ns'] = result['warmup_end_ns']+int(args.duration*1e9)
        next_progress = time.monotonic()+30
        while time.monotonic_ns() < result['measurement_end_ns']:
            for name in ('bridge','state'):
                if children[name].poll() is not None:
                    raise RuntimeError(f'{name} exited before measurement finished')
            if 'model' in children and children['model'].poll() not in (None,0):
                raise RuntimeError('Model load failed')
            rclpy.spin_once(node,timeout_sec=.005)
            if time.monotonic() >= next_progress:
                progress = {'phase':'measuring','elapsed_s':round((time.monotonic_ns()-result['start_mono_ns'])/1e9,1),
                            'imu_counts':imu_counts,'state_count':state_count,'observer_dropped':buffer.dropped,
                            'last_diagnostic':diagnostics[-1] if diagnostics else None}
                print(json.dumps(progress),flush=True)
                next_progress = time.monotonic()+30
        result['stop_requested_ns'] = time.monotonic_ns()
        # Stop the bridge only; continue state observation during trace flush and TTL expiry.
        try:
            os.killpg(children['bridge'].pid,signal.SIGINT)
        except ProcessLookupError:
            pass
        settle = time.monotonic()+1
        while time.monotonic() < settle:
            rclpy.spin_once(node,timeout_sec=.005)
        result['completed'] = True
    except (Exception, KeyboardInterrupt) as error:
        result['error'] = f'{type(error).__name__}: {error}'
    finally:
        result['end_mono_ns'] = time.monotonic_ns()
        if gc_event in gc.callbacks:
            gc.callbacks.remove(gc_event)
        cleanup_errors = {}
        for name in ('bridge','state','model','resources'):
            if name in children:
                proc = children[name]
                # Model/resources normally end at their own finite deadlines.
                if name in ('model','resources') and proc.poll() is None:
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        pass
                try:
                    stop_child(proc,grace=45 if name in ('bridge','state') else 5)
                except Exception as error:
                    cleanup_errors[name] = str(error)
        result['cleanup_errors'] = cleanup_errors
        result['child_exit_codes'] = {k:v.returncode for k,v in children.items()}
        result['observer_rows'] = buffer.size; result['observer_dropped'] = buffer.dropped
        result['imu_counts'] = imu_counts; result['state_count'] = state_count
        for f in logs:
            f.close()
        if legacy:
            legacy.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        buffer.dump(out/'observer.csv')
        (out/'diagnostics.json').write_text(json.dumps(diagnostics)+'\n')
        with (out/'device_times.csv').open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['entry_ns','device_ms']);w.writerows(device_times)
        try:
            result['port_after'] = check(args.device,args.motor_config)
        except Exception as error:
            result['port_after_error'] = str(error)
        result['completed'] = (result['completed'] and not buffer.dropped and not cleanup_errors
            and all(code == 0 for code in result['child_exit_codes'].values())
            and 'port_after_error' not in result)
        (out/'run.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k!='environment'},indent=2),flush=True)
    return 0 if result['completed'] else 1


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='mode',required=True)
    p=sub.add_parser('run')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--duration',type=float,default=600)
    p.add_argument('--warmup',type=float,default=30)
    p.add_argument('--trace',action='store_true')
    p.add_argument('--load',choices=['none','model'],default='none')
    p.add_argument('--observer-mode',choices=['buffered','legacy'],default='buffered')
    p.add_argument('--device',default='/dev/serial/by-id/usb-Livelybot_Livelybot_Communication_Board_2024051701-if08')
    p.add_argument('--motor-config',type=Path,default=Path(__file__).resolve().parents[2]/'config/parallel_exo/servo_pair.ref01.json')
    p.add_argument('--model-python',type=Path,default=Path('/home/nvidia/parallel_exo_ros_framework/output/mujoco-venv/bin/python'))
    p=sub.add_parser('resources');p.add_argument('--pids',required=True);p.add_argument('--duration',type=float,required=True);p.add_argument('--output',type=Path,required=True)
    p=sub.add_parser('model-worker');p.add_argument('--model-dir',type=Path,required=True);p.add_argument('--duration',type=float,required=True);p.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    max_duration=3600 if args.mode=='run' else 7200
    if not math.isfinite(args.duration) or not 1<=args.duration<=max_duration:
        parser.error(f'duration must be finite in [1,{max_duration}]')
    if args.mode=='run':
        if not math.isfinite(args.warmup) or not 0<=args.warmup<=120:
            parser.error('warmup must be finite in [0,120]')
        return run(args)
    return resource_worker(args) if args.mode=='resources' else model_worker(args)


if __name__=='__main__':
    raise SystemExit(main())
