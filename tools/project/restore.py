"""Restore this checkout's Python environment; never start hardware."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
LOCAL = ROOT / '.local'
MODEL = ROOT / 'mujoco' / 'ref01'


def environment():
    (LOCAL / 'tmp').mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env.update(PYTHONDONTWRITEBYTECODE='1', PYTHONIOENCODING='utf-8',
               PIP_CACHE_DIR=str(LOCAL / 'cache' / 'pip'),
               XDG_CACHE_HOME=str(LOCAL / 'cache'),
               MPLCONFIGDIR=str(LOCAL / 'cache' / 'matplotlib'),
               TMPDIR=str(LOCAL / 'tmp'), TEMP=str(LOCAL / 'tmp'), TMP=str(LOCAL / 'tmp'))
    return env


def run(args):
    subprocess.run([str(x) for x in args], cwd=ROOT, env=environment(), check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['inspect', 'setup-python', 'verify', 'example'])
    parser.add_argument('--python', default=sys.executable, help='Python 3.10 interpreter for creating the venv')
    parser.add_argument('--pip-python', help='Optional interpreter with pip >=22.3 to manage a pip-less venv')
    args = parser.parse_args()
    python = LOCAL / 'venv' / ('Scripts/python.exe' if os.name == 'nt' else 'bin/python')
    if args.action == 'inspect':
        print(json.dumps({'root': str(ROOT), 'python_environment': str(python),
                          'environment_exists': python.is_file(),
                          'model_exists': (MODEL / 'config/mujoco/ref01_dynamics_cad_v1.xml').is_file(),
                          'ros_setup_exists': Path('/opt/ros/humble/setup.bash').is_file(),
                          'hardware_started': False}, indent=2))
        return
    if args.action == 'setup-python':
        run([args.python, '-c', 'import sys; assert sys.version_info[:2] == (3,10), "Use Python 3.10"'])
        LOCAL.mkdir(exist_ok=True)
        if not python.exists():
            run([args.python, '-m', 'venv', *(['--without-pip'] if args.pip_python else []), LOCAL / 'venv'])
        pip = ([args.pip_python, '-m', 'pip', '--python', python]
               if args.pip_python else [python, '-m', 'pip'])
        run([*pip, 'install', '-r', ROOT / 'manifests/parallel_exo_python.lock.txt'])
        run([*pip, 'check'])
        return
    if not python.is_file():
        parser.error('Run setup-python with a Python 3.10 interpreter first')
    if args.action == 'verify':
        (MODEL / 'output').mkdir(exist_ok=True)
        run([python, '-I', '-B', MODEL / 'tools/verify_ref01_kinematics_api.py'])
        run([python, '-I', '-B', MODEL / 'tools/verify_ref01_dynamics_api.py'])
        run([python, '-I', '-B', ROOT / 'tools/project/verify_state_windows.py'])
    else:
        run([python, '-I', '-B', MODEL / 'tools/example_ref01_dynamics_api.py'])


if __name__ == '__main__':
    main()
