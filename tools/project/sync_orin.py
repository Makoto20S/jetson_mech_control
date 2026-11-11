"""One-way working-tree sync to an owned Orin development directory, via SSH."""
import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import shlex
import shutil
import stat
import subprocess
import sys
import tarfile
import uuid

EXCLUDED = ('.git', '.local', '.vscode', '.idea', 'memory', 'build', 'install',
            'log', 'output', 'tmp', '.cache', '.pytest_cache', 'ros2_ws/build',
            'ros2_ws/install', 'ros2_ws/log', 'mujoco/ref01/output', 'mujoco/ref01/tmp')


# Reports stay in Windows/Git. Keep them visible to the receiver so previously
# managed copies are hash-checked and backed up before removal from Orin.
LOCAL_ONLY_REPORTS = frozenset((
    'docs/parallel_exo/migration_validation.md',
    'docs/parallel_exo/migration_evidence.json',
    'docs/parallel_exo/imu_validation_20261106.md',
    'docs/parallel_exo/imu_timing_20261106.md',
    'docs/parallel_exo/mujoco_timing_20261106.md',
))
LOCAL_ONLY_REPORT_DIRECTORIES = ('reports', 'docs/parallel_exo/reports')


def local_only_report(name):
    return name in LOCAL_ONLY_REPORTS or any(
        name == prefix or name.startswith(prefix + '/')
        for prefix in LOCAL_ONLY_REPORT_DIRECTORIES)


def excluded(name):
    return (any(name == p or name.startswith(p + '/') for p in EXCLUDED)
            or '__pycache__' in PurePosixPath(name).parts or name.endswith('.pyc')
            or (name.startswith('config/parallel_exo/') and name.endswith('.local.json')))


def relative(name):
    p = PurePosixPath(name)
    if not name or p.is_absolute() or '\\' in name or ':' in name or any(
            part in ('', '.', '..') for part in name.split('/')) or excluded(name):
        raise RuntimeError('Disallowed source path: ' + name)
    return name


def safe_path(root, name=''):
    root = Path(root).absolute()
    path = root / name
    if not path.resolve().is_relative_to(root.resolve()):
        raise RuntimeError('Path escapes project: ' + str(path))
    for parent in [path, *path.parents]:
        if parent.exists() or parent.is_symlink():
            info = parent.lstat()
            if stat.S_ISLNK(info.st_mode) or getattr(info, 'st_file_attributes', 0) & 0x400:
                raise RuntimeError('Link/reparse point is not allowed: ' + str(parent))
    return path


def sha(data):
    return hashlib.sha256(data).hexdigest()


def stamp(files):
    return sha(json.dumps(files, sort_keys=True, separators=(',', ':'), ensure_ascii=False).encode())


def file_record(path, mode=None):
    data = path.read_bytes()
    return {'sha256': sha(data), 'size': len(data),
            'mode': mode if mode is not None else (0o755 if path.stat().st_mode & 0o111 else 0o644)}


def local_files(root):
    env = {**os.environ, 'GIT_OPTIONAL_LOCKS': '0'}
    def git(*args):
        return subprocess.check_output(['git', '-C', str(root), *args], env=env)
    if Path(git('rev-parse', '--show-toplevel').decode().strip()).resolve() != root.resolve():
        raise RuntimeError('Run from the inner jetson_mech_control Git repository')
    raw = git('ls-files', '-z', '--cached', '--others', '--exclude-standard')
    modes = {}
    for row in git('ls-files', '--stage', '-z').split(b'\0'):
        if row:
            meta, name = row.split(b'\t', 1)
            mode, _, stage = meta.split()
            if stage != b'0' or mode not in (b'100644', b'100755'):
                raise RuntimeError('Resolve Git conflicts, links or nested repositories before syncing')
            modes[name.decode()] = 0o755 if mode == b'100755' else 0o644
    result = {}
    for name in sorted(set(p.decode() for p in raw.split(b'\0') if p)):
        if excluded(name) or local_only_report(name):
            continue
        relative(name)
        path = safe_path(root, name)
        if not path.exists():  # Tracked files deleted in the working tree.
            continue
        if not path.is_file():
            raise RuntimeError('Not a regular source file: ' + name)
        result[name] = file_record(path, modes.get(name, 0o644))
    if not result or 'AGENTS.md' not in result:
        raise RuntimeError('Incomplete or unexpected source root')
    return result


def remote_files(root):
    result = {}
    if not root.exists():
        return result
    for directory, folders, files in os.walk(root, followlinks=False):
        base = Path(directory)
        folders[:] = [n for n in folders if not excluded((base / n).relative_to(root).as_posix())]
        for name in folders:
            safe_path(root, (base / name).relative_to(root).as_posix())
        for name in files:
            rel = (base / name).relative_to(root).as_posix()
            if not excluded(rel):
                relative(rel)
                path = safe_path(root, rel)
                if not path.is_file():
                    raise RuntimeError('Not a regular remote source: ' + rel)
                result[rel] = file_record(path)
    return result


def sync_plan(old, current, wanted, initialized):
    for name in set(old) | set(current) | set(wanted):
        relative(name)
    if not initialized and current:
        raise RuntimeError('Target has unregistered source files; use an empty development directory')
    conflicts = [p for p in old if current.get(p) != old[p] and current.get(p) != wanted.get(p)]
    conflicts += [p for p in current if p not in old]
    if conflicts:
        raise RuntimeError('Remote source changes must be reconciled locally first: ' + ', '.join(sorted(set(conflicts))[:12]))
    return {'update': sorted(p for p in wanted if current.get(p) != wanted[p]),
            'delete': sorted(set(old) - set(wanted))}


def atomic_json(path, value):
    safe_path(path.parent, path.name)
    temporary = path.with_name(path.name + '.tmp')
    safe_path(path.parent, temporary.name)
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    os.replace(temporary, path)


@contextmanager
def locked(root):
    import fcntl
    directory = safe_path(root, '.local')
    directory.mkdir(parents=True, exist_ok=True)
    lock = safe_path(root, '.local/sync.lock')
    with lock.open('a') as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Another sync/build/test is using this development tree') from None
        yield


def remote_operation(request):
    project = Path(request['project_root'])
    root = Path(request['remote_root'])
    if not project.is_absolute() or root != project / 'dev/jetson_mech_control':
        raise RuntimeError('Target must be <project_root>/dev/jetson_mech_control')
    safe_path(project)
    if not project.is_dir():
        raise RuntimeError('Project root does not exist')
    safe_path(project, 'dev/jetson_mech_control')
    action = request['action']
    wanted = request['files']
    for name, item in wanted.items():
        relative(name)
        if item['mode'] not in (0o644, 0o755) or not re.fullmatch('[0-9a-f]{64}', item['sha256']):
            raise RuntimeError('Invalid source manifest')
    if not root.exists() and action == 'status':
        return {'exists': False, 'files': {}, 'initialized': False, 'incomplete': False}
    if (root / '.git').exists():
        raise RuntimeError('Target is a Git checkout; this mirror does not overwrite remote Git work')
    root.mkdir(parents=True, exist_ok=True)
    with locked(root):
        metadata = safe_path(root, '.local/sync')
        metadata.mkdir(exist_ok=True)
        state_path = metadata / 'state.json'
        pending = metadata / 'incomplete.json'
        for name in ('state.json', 'incomplete.json', 'builds.json'):
            safe_path(root, '.local/sync/' + name)
        state = json.loads(state_path.read_text()) if state_path.exists() else None
        if state and state.get('root') != str(root):
            raise RuntimeError('Managed directory identity mismatch')
        current = remote_files(root)
        if action == 'status':
            return {'exists': True, 'initialized': bool(state), 'files': current,
                    'incomplete': pending.exists(), 'state_stamp': state.get('stamp') if state else None}
        if pending.exists():
            raise RuntimeError('Previous application was interrupted; inspect .local/sync/incomplete.json and its backup before continuing')
        old = state['files'] if state else {}
        plan = sync_plan(old, current, wanted, bool(state))
        if action == 'prepare':
            token = uuid.uuid4().hex
            incoming = safe_path(root, '.local/sync/incoming/' + token)
            incoming.mkdir(parents=True)
            return {**plan, 'token': token, 'archive': str(incoming / 'payload.tar.gz')}
        if action == 'apply':
            token = request['token']
            if not re.fullmatch('[0-9a-f]{32}', token):
                raise RuntimeError('Invalid transfer token')
            incoming = safe_path(root, '.local/sync/incoming/' + token)
            archive = safe_path(root, '.local/sync/incoming/' + token + '/payload.tar.gz')
            if sha(archive.read_bytes()) != request['archive_sha256']:
                raise RuntimeError('Uploaded archive checksum mismatch; original sources preserved')
            staged = incoming / 'staged'
            safe_path(root, staged.relative_to(root).as_posix()).mkdir()
            with tarfile.open(archive, 'r:gz') as tar:
                members = tar.getmembers()
                if len(members) != len(plan['update']) or {m.name for m in members} != set(plan['update']):
                    raise RuntimeError('Archive file list differs from the validated update plan')
                for member in members:
                    if not member.isfile() or member.size != wanted[member.name]['size']:
                        raise RuntimeError('Invalid archive entry: ' + member.name)
                    data = tar.extractfile(member).read()
                    if sha(data) != wanted[member.name]['sha256']:
                        raise RuntimeError('File checksum mismatch: ' + member.name)
                    destination = safe_path(staged, relative(member.name))
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    destination.write_bytes(data)
                    destination.chmod(wanted[member.name]['mode'])
            if remote_files(root) != current:
                raise RuntimeError('Remote sources changed while staging; application cancelled')
            converted_directories = []
            for name in plan['update']:
                destination = safe_path(root, name)
                if destination.is_dir():
                    for child in destination.rglob('*'):
                        child_name = child.relative_to(root).as_posix()
                        safe_path(root, child_name)
                        if child.is_file() and child_name not in plan['delete']:
                            raise RuntimeError('Directory replacement would remove an unmanaged file: ' + child_name)
                    converted_directories.append(destination)
            backup = incoming / 'before'
            for name in plan['update'] + plan['delete']:
                source = safe_path(root, name)
                if source.is_file():
                    destination = safe_path(backup, name)
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(source, destination)
            atomic_json(pending, {'token': token, 'plan': plan, 'old_stamp': state.get('stamp') if state else None})
            for name in plan['delete']:
                path = safe_path(root, name)
                if path.exists():
                    path.unlink()
            for directory in converted_directories:
                for child in sorted(directory.rglob('*'), key=lambda p: len(p.parts), reverse=True):
                    safe_path(root, child.relative_to(root).as_posix())
                    if child.is_dir():
                        child.rmdir()  # Only empty directories; never recursively remove files.
                directory.rmdir()
            for name in plan['update']:
                destination = safe_path(root, name)
                destination.parent.mkdir(parents=True, exist_ok=True)
                os.replace(safe_path(staged, name), destination)
            if remote_files(root) != wanted:
                raise RuntimeError('Final source verification failed; incomplete marker retained')
            state = {'schema': 1, 'root': str(root), 'stamp': stamp(wanted), 'files': wanted,
                     'synced_at': datetime.now(timezone.utc).isoformat(), 'source_git': request['source_git']}
            atomic_json(state_path, state)
            pending.unlink()
            return {'status': 'synchronized', 'files': len(wanted), 'stamp': state['stamp'],
                    'updated': len(plan['update']), 'deleted': len(plan['delete'])}
        if action in ('build', 'test'):
            if not state or current != wanted or state['stamp'] != stamp(wanted):
                raise RuntimeError('Local and Orin sources differ; synchronize first')
            profile = request['profile']
            if profile not in ('imu', 'state', 'servo_imu', 'all'):
                raise RuntimeError('Invalid build profile')
            build_path = metadata / 'builds.json'
            builds = json.loads(build_path.read_text()) if build_path.exists() else {}
            if action == 'test' and builds.get(profile) != state['stamp']:
                raise RuntimeError('Build this exact source snapshot before testing')
            if action == 'build':
                builds.pop(profile, None)
                atomic_json(build_path, builds)
            log = metadata / (action + '-' + profile + '-' + uuid.uuid4().hex + '.log')
            env = {**os.environ, 'EXO_SYNC_LOCK_HELD': '1', 'CMAKE_BUILD_PARALLEL_LEVEL': '2', 'MAKEFLAGS': '-j2'}
            with log.open('w') as output:
                result = subprocess.run(['bash', 'tools/project/ros_workspace.sh', action, profile],
                                        cwd=root, env=env, stdout=output, stderr=subprocess.STDOUT)
            if remote_files(root) != wanted:
                raise RuntimeError('Sources changed during build/test; results are invalid')
            if action == 'build' and result.returncode == 0:
                builds[profile] = state['stamp']
                atomic_json(build_path, builds)
            return {'status': action, 'exit_code': result.returncode, 'profile': profile,
                    'stamp': state['stamp'], 'log': str(log), 'tail': '\n'.join(log.read_text(errors='replace').splitlines()[-25:])}
        raise RuntimeError('Unknown remote operation')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['status', 'sync', 'build', 'test'], nargs='?', default='status')
    parser.add_argument('--profile', choices=['imu', 'state', 'servo_imu', 'all'], default='imu')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    config = json.loads((root / 'config/parallel_exo/orin_sync.json').read_text(encoding='utf-8-sig'))
    host = config['ssh_host']
    if not re.fullmatch('[A-Za-z0-9_][A-Za-z0-9_.@-]*', host):
        raise RuntimeError('Use a configured SSH alias or user@host without shell characters')
    options = ['-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=10', '-o', 'ConnectionAttempts=1']
    if args.action == 'sync':
        subprocess.run([sys.executable, '-B', str(root / 'tools/project/verify_state_windows.py')], check=True)
    files = local_files(root)
    base = {**config, 'files': files}
    def rpc(action, **extra):
        request = {**base, 'action': action, **extra}
        code = ('_EXO_REQUEST = ' + repr(json.dumps(request)) + '\n').encode() + Path(__file__).read_bytes()
        result = subprocess.run(['ssh', '-T', *options, host, 'python3 -B -'], input=code,
                                capture_output=True, timeout=1800 if action in ('build', 'test') else 120)
        if result.returncode:
            raise RuntimeError(result.stderr.decode('utf-8', errors='replace').strip() or 'SSH operation failed')
        return json.loads(result.stdout.decode('utf-8'))
    report = safe_path(root, '.local/sync')
    report.mkdir(parents=True, exist_ok=True)
    if args.action == 'status':
        remote = rpc('status')
        difference = sorted(p for p in set(files) | set(remote['files']) if files.get(p) != remote['files'].get(p))
        result = {'status': 'equal' if not difference and remote['initialized'] and not remote['incomplete'] else 'different',
                  'local_files': len(files), 'remote_files': len(remote['files']), 'different_paths': difference,
                  'local_stamp': stamp(files), 'remote_root': config['remote_root'], 'incomplete': remote['incomplete']}
    elif args.action == 'sync':
        plan = rpc('prepare')
        directory = safe_path(root, '.local/sync/' + plan['token'])
        directory.mkdir()
        archive = directory / 'payload.tar.gz'
        with tarfile.open(archive, 'w:gz') as tar:
            for name in plan['update']:
                data = safe_path(root, name).read_bytes()
                if sha(data) != files[name]['sha256']:
                    raise RuntimeError('Local source changed during packaging; synchronize again after editing')
                info = tarfile.TarInfo(name)
                info.size, info.mode = len(data), files[name]['mode']
                tar.addfile(info, io.BytesIO(data))
        if local_files(root) != files:
            raise RuntimeError('Local file list/content changed during packaging; no remote source applied')
        subprocess.run(['scp', *options, str(archive), host + ':' + shlex.quote(plan['archive'])], check=True, timeout=120)
        if local_files(root) != files:
            raise RuntimeError('Local sources changed during upload; no remote source applied')
        env = {**os.environ, 'GIT_OPTIONAL_LOCKS': '0'}
        head = subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], env=env).decode().strip()
        result = rpc('apply', token=plan['token'], archive_sha256=sha(archive.read_bytes()), source_git={'head': head, 'working_tree_included': True})
        if local_files(root) != files:
            raise RuntimeError('Snapshot was synchronized, but local editing continued; run sync again before build')
    else:
        print('Running ' + args.action + ' on Orin: ' + config['remote_root'], flush=True)
        result = rpc(args.action, profile=args.profile)
        result['local_changed_after_start'] = local_files(root) != files
    atomic_json(report / 'last_result.json', result)
    print(json.dumps({k: v for k, v in result.items() if k != 'tail'}, ensure_ascii=False, indent=2))
    if 'tail' in result:
        print(result['tail'])
    if result.get('exit_code', 0):
        raise SystemExit(result['exit_code'])
    if result.get('local_changed_after_start'):
        raise RuntimeError('Local editing continued during build/test; result applies only to the recorded snapshot')


if __name__ == '__main__':
    try:
        if '_EXO_REQUEST' in globals():
            print(json.dumps(remote_operation(json.loads(_EXO_REQUEST)), ensure_ascii=False))
        else:
            main()
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.SubprocessError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
