"""Check the selected sensor endpoint without opening it or changing permissions."""
import json
import os
from pathlib import Path
import subprocess
import sys

DEFAULT_DEVICE = '/dev/serial/by-id/usb-Livelybot_Livelybot_Communication_Board_2024051701-if08'


def check(device, motor_config):
    path = Path(device)
    if not path.exists() or not path.is_char_device():
        raise RuntimeError(f'Sensor device is absent or not a character device: {device}')
    resolved = path.resolve()
    config = json.loads(Path(motor_config).read_text())
    motors = {Path(bus['device_path']).resolve() for bus in config['buses']}
    if resolved in motors:
        raise RuntimeError(f'Selected port belongs to the motor configuration: {resolved}')
    if not os.access(path, os.R_OK | os.W_OK):
        raise RuntimeError(f'Current user cannot access the sensor port: {device}')
    owners = subprocess.run(['fuser', str(resolved)], capture_output=True, text=True)
    if owners.returncode != 1 or owners.stdout.strip() or owners.stderr.strip():
        raise RuntimeError(f'Port is owned or ownership check failed: {owners.stdout} {owners.stderr}')
    return {'selected': device, 'resolved': str(resolved), 'motor_ports': sorted(map(str, motors))}


if __name__ == '__main__':
    try:
        print(json.dumps(check(sys.argv[1], sys.argv[2])))
    except (OSError, ValueError, KeyError, IndexError, RuntimeError) as error:
        print(str(error), file=sys.stderr)
        sys.exit(1)
