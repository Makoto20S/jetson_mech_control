"""Non-hardware timing/resource benchmark for the repository REF01 Python API.\n\nAdapted from Mujoco-Win/tools/benchmark_ref01_orin.py (2026-11-03).\nCOMPLETED records collection completion, not a real-time acceptance decision.

Supervisor samples /proc and starts its own tegrastats process. Worker constructs
one model, warms it up, and records every call; no changes to system scheduling.
"""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time


def write_json(path, obj):
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2), encoding='utf-8')


def snapshot():
    frequencies = {}
    for path in sorted(Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq')):
        frequencies[path.parent.name] = {n: (path / n).read_text().strip() for n in
            ('scaling_governor', 'scaling_cur_freq', 'scaling_min_freq', 'scaling_max_freq') if (path / n).exists()}
    temperatures = {}
    for path in Path('/sys/class/thermal').glob('thermal_zone*'):
        try:
            temperatures[(path / 'type').read_text().strip()] = int((path / 'temp').read_text()) / 1000
        except OSError:
            pass
    return dict(timestamp=datetime.now().astimezone().isoformat(), kernel=platform.release(),
                cpu_affinity=sorted(os.sched_getaffinity(0)), load_average=os.getloadavg(),
                cpu_frequencies=frequencies, temperatures_C=temperatures,
                realtime_kernel_flag=Path('/sys/kernel/realtime').read_text().strip()
                    if Path('/sys/kernel/realtime').exists() else 'not exposed',
                scheduler=os.sched_getscheduler(0), nice=os.getpriority(os.PRIO_PROCESS, 0))


def process_sample(pid):
    try:
        path = Path('/proc') / str(pid)
        fields = (path / 'stat').read_text().split(') ', 1)[1].split()
        status = dict(line.split(':', 1) for line in (path / 'status').read_text().splitlines() if ':' in line)
        return dict(cpu_ticks=int(fields[11]) + int(fields[12]),
                    rss_kib=int(status.get('VmRSS', '0 kB').split()[0]),
                    peak_rss_kib=int(status.get('VmHWM', '0 kB').split()[0]),
                    threads=int(status['Threads']))
    except (OSError, KeyError, IndexError):
        return None


def system_ticks():
    values = list(map(int, Path('/proc/stat').read_text().splitlines()[0].split()[1:9]))
    return sum(values), values[3] + values[4]



def validate_paths(repository, release, output):
    repository = Path(repository).resolve()
    release = Path(release).resolve()
    output = Path(output).resolve()
    expected = repository / 'mujoco/ref01'
    allowed = repository / '.local/validation'
    if release != expected or not release.is_dir():
        raise ValueError('release must be this repository mujoco/ref01 directory')
    if output == allowed or not output.is_relative_to(allowed):
        raise ValueError('output must be a fresh subdirectory of repository .local/validation')
    if output.exists():
        raise FileExistsError('Preserve previous benchmark results')
    return release, output


def model_inputs(release):
    required = [release / name for name in (
        'ref01_dynamics_api.py', 'ref01_kinematics.py', 'tools/ref01_v1.py',
        'config/mujoco/ref01_dynamics_cad_v1.xml')]
    if not all(path.is_file() for path in required):
        raise FileNotFoundError('Incomplete REF01 model/API input files')
    paths = sorted(set(required + [p for p in (release / 'config').rglob('*') if p.is_file()]))
    for path in paths:
        if not path.resolve().is_relative_to(release) or path.is_symlink():
            raise ValueError('Model inputs must remain within the source tree')
    return paths


def next_periodic_slot(current_slot, finish_ns, epoch_ns, period_ns):
    if period_ns <= 0:
        raise ValueError('period_ns must be positive')
    return max(current_slot + 1, (finish_ns - epoch_ns + period_ns - 1) // period_ns)


def stop_owned_process(process):
    """Stop only the process handle created by this supervisor."""
    if process is None:
        return
    if process.poll() is None:
        try:
            process.terminate()
        except ProcessLookupError:
            pass
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            process.kill()
        except ProcessLookupError:
            pass
        process.wait(timeout=5)


def worker(args):
    import contextlib
    import ctypes
    import gc
    import io
    import resource
    import numpy as np
    import mujoco
    sys.path.insert(0, str(args.release))
    from ref01_dynamics_api import REF01Dynamics
    out = args.output
    raw = {}

    def phase(name):
        write_json(out / 'phase.json', dict(phase=name))
        print('PHASE ' + name, flush=True)

    def stats(values_ns):
        a = np.asarray(values_ns, dtype=np.float64) / 1000
        return dict(count=len(a), mean_us=float(a.mean()), min_us=float(a.min()),
                    p50_us=float(np.percentile(a, 50)), p95_us=float(np.percentile(a, 95)),
                    p99_us=float(np.percentile(a, 99)), p999_us=float(np.percentile(a, 99.9)),
                    max_us=float(a.max()), over_1ms=int(np.count_nonzero(a > 1000)),
                    over_2ms=int(np.count_nonzero(a > 2000)))

    def usage(before, wall):
        after = resource.getrusage(resource.RUSAGE_SELF)
        return dict(wall_s=wall, cpu_s=(after.ru_utime+after.ru_stime)-(before.ru_utime+before.ru_stime),
                    cpu_one_core_percent=100*((after.ru_utime+after.ru_stime)-(before.ru_utime+before.ru_stime))/wall,
                    voluntary_context_switches=after.ru_nvcsw-before.ru_nvcsw,
                    involuntary_context_switches=after.ru_nivcsw-before.ru_nivcsw,
                    minor_page_faults=after.ru_minflt-before.ru_minflt, major_page_faults=after.ru_majflt-before.ru_majflt,
                    peak_rss_kib=after.ru_maxrss)

    phase('idle_baseline')
    time.sleep(3)
    before_load = process_sample(os.getpid())
    t0 = time.perf_counter_ns()
    robot = REF01Dynamics()
    init_ms = (time.perf_counter_ns()-t0)/1e6
    after_load = process_sample(os.getpid())
    backend = []
    libs = Path(np.__file__).resolve().parents[1] / 'numpy.libs'
    for path in libs.glob('*openblas*.so*'):
        lib = ctypes.CDLL(str(path))
        for name in ('scipy_openblas_get_num_threads64_', 'scipy_openblas_get_num_threads',
                     'openblas_get_num_threads64_', 'openblas_get_num_threads'):
            fn = getattr(lib, name, None)
            if fn is not None:
                fn.restype = ctypes.c_int
                backend.append(dict(library=path.name, function=name, threads=fn()))
                break
    phase('prepare_and_validate')
    rng = np.random.default_rng(20261103)
    q = np.deg2rad(rng.uniform([-44, 1], [44, 44], (1024, 2)))
    dq = rng.uniform(-.8, .8, (1024, 2))
    ddq = rng.uniform(-2, 2, (1024, 2))
    zeros = np.zeros(2)
    tau = np.asarray([robot.inverse_dynamics(q[i], dq[i], ddq[i]) for i in range(len(q))])
    static_tau = np.asarray([robot.gravity_compensation(x) for x in q])
    for i in range(32):
        assert np.max(np.abs(robot.forward_dynamics(q[i], dq[i], tau[i])-ddq[i])) < 1e-9
    inverse_args = [(q[i], dq[i], ddq[i]) for i in range(len(q))]
    forward_args = [(q[i], dq[i], tau[i]) for i in range(len(q))]
    jobs = [('inverse_dynamic', robot.inverse_dynamics, inverse_args),
            ('forward_dynamic', robot.forward_dynamics, forward_args),
            ('inverse_static', robot.inverse_dynamics, [(x, zeros, zeros) for x in q]),
            ('forward_static', robot.forward_dynamics, [(q[i], zeros, static_tau[i]) for i in range(len(q))])]
    for _, method, requests in jobs:
        for i in range(512):
            method(*requests[i])
    timer = np.empty(10000, dtype=np.int64)
    for i in range(len(timer)):
        start = time.perf_counter_ns()
        timer[i] = time.perf_counter_ns()-start
    result = dict(status='RUNNING', label=args.label, environment=snapshot(),
                  python=sys.version, python_executable=sys.executable, mujoco=mujoco.__version__, numpy=np.__version__,
                  blas_threads_env=args.blas_threads, detected_blas=backend,
                  init_ms=init_ms, before_model=before_load, after_model=after_load,
                  model=robot.metadata, timer_pair=stats(timer), gc_enabled=gc.isenabled(),
                  input_definition=dict(samples=1024, seed=20261103, psi_deg=[-44, 44], beta_deg=[1, 44],
                                        velocity_rad_s=[-.8, .8], acceleration_rad_s2=[-2, 2]),
                  micro={}, periodic={}, batches={}, physical_hardware_used=False)
    if args.mode in ('micro', 'all'):
        for name, method, requests in jobs:
            phase(name)
            a = np.empty(args.count, dtype=np.int64)
            before = resource.getrusage(resource.RUSAGE_SELF)
            start_wall = time.perf_counter()
            for i in range(len(a)):
                request = requests[i & 1023]
                start = time.perf_counter_ns()
                last = method(*request)
                a[i] = time.perf_counter_ns()-start
            wall = time.perf_counter()-start_wall
            assert np.isfinite(last).all()
            result['micro'][name] = dict(**stats(a), **usage(before, wall))
            raw[name+'_ns'] = a
            print(json.dumps(dict(phase=name, **result['micro'][name])), flush=True)

    if args.mode in ('periodic', 'all'):
        for hz in args.rates:
            name = f'inverse_periodic_{hz}Hz'
            phase(name)
            period = int(1e9/hz)
            slots = int(args.seconds*hz)
            data = np.empty((slots, 4), dtype=np.int64)
            k = 0
            used = 0
            before = resource.getrusage(resource.RUSAGE_SELF)
            start_wall = time.perf_counter()
            epoch = time.perf_counter_ns()+5_000_000
            while k < slots:
                target = epoch+k*period
                remaining = target-time.perf_counter_ns()
                if remaining > 0:
                    time.sleep(remaining/1e9)
                request = inverse_args[k & 1023]
                begin = time.perf_counter_ns()
                last = robot.inverse_dynamics(*request)
                end = time.perf_counter_ns()
                data[used] = (k, begin-target, end-begin, end-target)
                used += 1
                # Skip releases that are already in the past; never send a burst of stale jobs.
                k = next_periodic_slot(k, end, epoch, period)
            wall = time.perf_counter()-start_wall
            data = data[:used]
            metrics = dict(compute=stats(data[:, 2]), wake_lateness=stats(data[:, 1]),
                           release_to_finish=stats(data[:, 3]), scheduled_slots=slots, issued_calls=used,
                           skipped_releases=slots-used, deadline_miss_calls=int(np.count_nonzero(data[:, 3] > period)),
                           period_us=period/1000, target_duration_s=args.seconds,
                           **usage(before, wall))
            result['periodic'][str(hz)] = metrics
            raw[name+'_slot_wake_compute_finish_ns'] = data
            print(json.dumps(dict(phase=name, **metrics)), flush=True)

    if args.mode in ('batch', 'all'):
        for calls in (20, 50, 100):
            phase(f'forward_batch_{calls}')
            a = np.empty(args.batch_repeats, dtype=np.int64)
            before = resource.getrusage(resource.RUSAGE_SELF)
            start_wall = time.perf_counter()
            for repeat in range(len(a)):
                start = time.perf_counter_ns()
                for i in range(calls):
                    last = robot.forward_dynamics(*forward_args[(repeat*calls+i) & 1023])
                a[repeat] = time.perf_counter_ns()-start
            wall = time.perf_counter()-start_wall
            result['batches'][str(calls)] = dict(**stats(a), **usage(before, wall),
                purpose='Sequential full API evaluation budget; not an MPC solver or trajectory rollout')
            raw[f'forward_batch_{calls}_ns'] = a
            print(json.dumps(dict(phase=f'batch_{calls}', **result['batches'][str(calls)])), flush=True)

    phase('complete')
    result['status'] = 'COMPLETED'
    result['end_environment'] = snapshot()
    np.savez_compressed(out / 'timings.npz', **raw)
    write_json(out / 'worker_result.json', result)



def supervise(args):
    repository = Path(__file__).resolve().parents[2]
    args.release, args.output = validate_paths(repository, args.release, args.output)
    watched = model_inputs(args.release)
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in watched}
    args.output.mkdir(parents=True)
    write_json(args.output / 'environment_before.json', snapshot())
    found = shutil.which('nvpmodel')
    if found:
        p = subprocess.run([found, '-q'], capture_output=True, text=True)
        (args.output / 'nvpmodel.txt').write_text(p.stdout+p.stderr, encoding='utf-8')
    env = os.environ.copy()
    env.update(PYTHONUNBUFFERED='1', PYTHONDONTWRITEBYTECODE='1',
               TMPDIR=str(args.output), XDG_CACHE_HOME=str(repository/'.local/cache'))
    if args.blas_threads:
        for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
            env[name] = str(args.blas_threads)
    command = [sys.executable, '-I', '-B', '-X', 'utf8', str(Path(__file__).resolve()), '--worker',
               '--release', str(args.release), '--output', str(args.output), '--label', args.label,
               '--mode', args.mode, '--count', str(args.count), '--seconds', str(args.seconds),
               '--batch-repeats', str(args.batch_repeats), '--blas-threads', str(args.blas_threads),
               '--rates', *map(str, args.rates)]
    tegra = child = None
    with (args.output/'tegrastats.log').open('w') as ts, (args.output/'worker.log').open('w') as log:
        previous = {}
        old_sys = system_ticks()
        old_time = time.monotonic()
        tick_hz = os.sysconf('SC_CLK_TCK')
        try:
            if shutil.which('tegrastats'):
                tegra = subprocess.Popen(['tegrastats', '--interval', '200'], stdout=ts,
                                         stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
            child = subprocess.Popen(command, env=env, cwd=args.output, stdout=log,
                                     stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
            with (args.output/'resources.jsonl').open('w') as stream:
                while child.poll() is None:
                    now = time.monotonic()
                    dt = now-old_time
                    try:
                        phase = json.loads((args.output/'phase.json').read_text())['phase']
                    except (OSError, ValueError):
                        phase = 'startup'
                    row = dict(t_monotonic_s=now, phase=phase)
                    for name, pid in (('worker', child.pid), ('viewer', args.viewer_pid)):
                        state = process_sample(pid) if pid else None
                        if state and name in previous and dt > 0:
                            state['cpu_one_core_percent'] = 100*(state['cpu_ticks']-previous[name]['cpu_ticks'])/tick_hz/dt
                        row[name] = state
                        if state:
                            previous[name] = state
                    system = system_ticks()
                    total = system[0]-old_sys[0]
                    row['system_cpu_all_cores_percent'] = 100*(1-(system[1]-old_sys[1])/total) if total else 0
                    stream.write(json.dumps(row)+'\n')
                    old_time, old_sys = now, system
                    time.sleep(.2)
        finally:
            # Even interrupted or failed collection must release both owned children.
            try:
                stop_owned_process(child)
            finally:
                stop_owned_process(tegra)
        if child.returncode:
            print((args.output/'worker.log').read_text(), flush=True)
            raise RuntimeError(f'Worker exit {child.returncode}')
    for path, expected in hashes.items():
        assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == expected, path
    result = json.loads((args.output/'worker_result.json').read_text())
    result['frozen_files_unchanged'] = hashes
    result['viewer_pid'] = args.viewer_pid
    result['benchmark_source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    result['measurement_notes'] = [
        'One REF01Dynamics object; loading and warmup excluded from per-call latency.',
        'Dynamic requests vary pose, nonzero speed and acceleration; static cases use zero velocity.',
        'Normal scheduler, default GC, no affinity/power/governor changes; only requested worker BLAS env changes.',
        'Raw timings include preemption; /proc sampled externally every 200 ms; tegrastats sampled every 200 ms.',
        'CPU percent uses one core=100%; system CPU percent uses all online cores=100%.',
        'Periodic tests use absolute releases with Python time.sleep; late releases are skipped.',
        'No sensor, ROS messaging, actuator communication, optimizer, or hardware commands are included.',
        'COMPLETED means collection finished; deadline_miss_calls and skipped_releases must be assessed separately.']
    write_json(args.output/'result.json', result)
    print(json.dumps(dict(status='COMPLETED', label=args.label, output=str(args.output),
                          detected_blas=result['detected_blas'], micro=result['micro'],
                          periodic=result['periodic'], batches=result['batches']), indent=2), flush=True)



if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--release', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--label', required=True)
    p.add_argument('--mode', choices=('micro','periodic','batch','all'), default='micro')
    p.add_argument('--worker', action='store_true')
    p.add_argument('--count', type=int, default=20000)
    p.add_argument('--seconds', type=float, default=60)
    p.add_argument('--rates', type=int, nargs='+', default=[500, 1000])
    p.add_argument('--batch-repeats', type=int, default=200)
    p.add_argument('--blas-threads', type=int, default=0)
    p.add_argument('--viewer-pid', type=int, default=0)
    args = p.parse_args()
    if min(args.count, args.seconds, args.batch_repeats, *args.rates) <= 0:
        p.error('Counts and rates must be positive')
    worker(args) if args.worker else supervise(args)
