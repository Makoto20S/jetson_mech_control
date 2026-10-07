"""Bounded, shutdown-only diagnostic storage; never opens a device."""
import argparse
import gzip
import json
import math
import os
from pathlib import Path
import shutil
import struct
import tempfile
import threading
import uuid

MIB = 1024 * 1024
SESSION_LIMIT = 128 * MIB
ROOT_LIMIT = 2048 * MIB
FREE_RESERVE = 1024 * MIB
CHAIN_CAPACITY = 4194304
RAW_CAPACITY = 524288
SNAPSHOT_LIMIT = 1536 * MIB
HEADER_BYTES = 128


def directory_bytes(path):
    # Count every regular file, including old evidence; never follow directory links.
    return sum(p.stat().st_size for p in Path(path).rglob('*')
               if p.is_file() and not p.is_symlink()) if Path(path).exists() else 0


def admit(run, additional=SESSION_LIMIT):
    run = Path(run)
    base = run.parent
    base.mkdir(parents=True, exist_ok=True)
    used = directory_bytes(run)
    additional = min(additional, SESSION_LIMIT - used)
    if additional <= 16 * MIB:
        raise ValueError('本次会话128 MiB日志额度不足；请退出并手工归档日志')
    if directory_bytes(base) + additional > ROOT_LIMIT:
        raise ValueError('日志目录达到2 GiB额度；已有证据不会自动删除，请手工归档')
    if shutil.disk_usage(base).free < FREE_RESERVE + additional:
        raise ValueError('磁盘可用空间不足：必须保留1 GiB及本次日志额度；拒绝启动设备')
    return additional


class LimitedFile:
    """Hard byte cap, including gzip headers/trailer. No uncompressed intermediary."""
    def __init__(self, path, limit):
        self.stream = Path(path).open('xb')
        self.limit, self.size = limit, 0
    def write(self, data):
        if self.size + len(data) > self.limit:
            raise OSError('compressed/file byte quota exhausted')
        count = self.stream.write(data)
        self.size += count
        return count
    def flush(self):
        self.stream.flush()
    def close(self):
        self.stream.close()


class BoundedEvents:
    def __init__(self, path, limit=2*MIB):
        self.stream = Path(path).open('x', encoding='utf-8', buffering=1, newline='\n')
        self.limit, self.size, self.dropped = limit, 0, 0
        self.error = None
    def write(self, text):
        size = len(text.encode('utf-8'))
        if self.error or self.size + size > self.limit:
            self.dropped += 1
            return
        try:
            self.stream.write(text)
            self.size += size
        except OSError as exc:
            self.error = str(exc)
            self.dropped += 1
    def close(self):
        self.stream.close()


class FrameworkDrain:
    """Keep draining even after quota/disk failure; child cannot block on a full pipe."""
    def __init__(self, pipe, path, limit=4*MIB):
        self.pipe, self.path, self.limit = pipe, Path(path), limit
        self.size = self.discarded = 0
        self.error = None
        self.thread = threading.Thread(target=self._drain, daemon=True)
        self.thread.start()
    def _drain(self):
        output = None
        try:
            try:
                output = self.path.open('xb')
            except OSError as exc:
                self.error = str(exc)
            while True:
                data = self.pipe.read(65536)
                if not data:
                    break
                accepted = min(len(data), max(0, self.limit-self.size)) if output and not self.error else 0
                if accepted:
                    try:
                        output.write(data[:accepted]); self.size += accepted
                    except OSError as exc:
                        self.error = str(exc); accepted = 0
                self.discarded += len(data)-accepted
        finally:
            if output:
                try:
                    output.close()
                except OSError as exc:
                    self.error = str(exc)
            self.pipe.close()
    def close(self):
        self.thread.join(2.)
        if self.thread.is_alive():
            raise RuntimeError('framework output pipe remains open')


def capture_bytes(chain_capacity, raw_capacity):
    size = HEADER_BYTES*2 + chain_capacity*216 + raw_capacity*1064
    if chain_capacity <= 0 or raw_capacity <= 0 or size > SNAPSHOT_LIMIT:
        raise ValueError('无效记录容量：快照总额必须在1536 MiB以内')
    return size


def make_snapshots(run, chain_capacity=CHAIN_CAPACITY, raw_capacity=RAW_CAPACITY, session_root=None):
    size = capture_bytes(chain_capacity, raw_capacity)
    shm = Path('/dev/shm')
    mounts = Path('/proc/mounts').read_text().splitlines()
    if not any(len(row.split()) > 2 and row.split()[1:3] == ['/dev/shm', 'tmpfs'] for row in mounts):
        raise ValueError('/dev/shm不是已确认的tmpfs；拒绝启动设备')
    available = None
    for line in Path('/proc/meminfo').read_text().splitlines():
        if line.startswith('MemAvailable:'):
            available = int(line.split()[1])*1024
    if available is None or available < size + 256*MIB or shutil.disk_usage(shm).free < size + 16*MIB:
        raise ValueError(f'RAM或/dev/shm不足：记录需要{math.ceil(size/MIB)} MiB并保留256 MiB RAM；拒绝启动设备')
    # A previous crashed capture is evidence, not disposable scratch data.
    for candidate in shm.glob('servo-trace-*'):
        marker = candidate/'owner.json'
        if marker.is_file():
            try:
                owner = json.loads(marker.read_text())
                if owner.get('uid') == os.getuid() and Path(owner.get('run_root', Path(owner['run']).parent)).resolve() == Path(session_root or run).parent.resolve():
                    raise ValueError('存在未恢复快照 '+str(candidate)+'；请先用trace_storage.py离线恢复')
            except (OSError, KeyError, json.JSONDecodeError):
                pass
    path = Path(tempfile.mkdtemp(prefix='servo-trace-', dir=shm))
    (path/'owner.json').write_text(json.dumps(dict(uid=os.getuid(), run=str(Path(run).resolve()),
        run_root=str(Path(session_root or run).parent.resolve())))+'\n')
    return path


def snapshot_metadata(path, expected_kind):
    with Path(path).open('rb') as stream:
        data = stream.read(HEADER_BYTES)
        if len(data) != HEADER_BYTES or data[:8] != b'MCHTRC01':
            raise ValueError('invalid snapshot header')
        version, kind, stride, header = struct.unpack_from('<4I', data, 8)
        capacity, total, committed, dropped, overwritten, closed = struct.unpack_from('<6Q', data, 24)
        if (version != 1 or kind != expected_kind or header != HEADER_BYTES or
                stride != (216 if kind == 1 else 1064) or capacity < 1 or
                committed != min(total, capacity) or closed not in (0,1) or
                dropped != (max(total-capacity,0) if kind == 1 else 0) or
                overwritten != (max(total-capacity,0) if kind == 2 else 0) or
                any(data[72:]) or Path(path).stat().st_size != header+capacity*stride):
            raise ValueError('invalid snapshot layout/counters')
    return dict(schema_version=1, capacity=capacity, total=total, committed=committed,
                dropped=dropped, overwritten=overwritten, closed=bool(closed), kind=kind, stride=stride)


def register_writer(snapshot, pid):
    marker = Path(snapshot)/'owner.json'
    owner = json.loads(marker.read_text())
    owner['writer_pid'] = pid
    try:
        owner['writer_start'] = Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19]
    except FileNotFoundError:
        owner['writer_start'] = None
    marker.write_text(json.dumps(owner)+'\n')


def abandon_prelaunch(snapshot, run):
    """Only remove a freshly owned empty directory when Popen never succeeded."""
    snapshot = Path(snapshot)
    owner = json.loads((snapshot/'owner.json').read_text())
    if (owner.get('writer_pid') is not None or owner['uid'] != os.getuid() or
            Path(owner['run']).resolve() != Path(run).resolve() or
            {p.name for p in snapshot.iterdir()} != {'owner.json'}):
        raise ValueError('not an empty, owned prelaunch snapshot')
    shutil.rmtree(snapshot)


def require_writer_exited(owner):
    pid = owner.get('writer_pid')
    if not isinstance(pid,int) or pid <= 0:
        raise ValueError('writer identity absent; cannot confirm offline recovery safety')
    try:
        # Check the group even if the launcher leader already exited.
        os.killpg(pid,0)
    except ProcessLookupError:
        return
    raise ValueError('snapshot writer process group is still present; refuse recovery')


def snapshot_rows(path, meta):
    with Path(path).open('rb') as stream:
        start = meta['total']-meta['committed'] if meta['kind'] == 2 else 0
        end = start+meta['committed']
        if meta['kind'] == 2 and not meta['closed'] and meta['total'] >= meta['capacity']:
            # The oldest ring slot may have been overwritten before publication.
            start += 1
        for seq in range(start, end):
            slot = seq % meta['capacity']
            stream.seek(HEADER_BYTES+slot*meta['stride'])
            data = stream.read(meta['stride'])
            if len(data) != meta['stride']:
                raise ValueError('truncated snapshot record')
            if meta['kind'] == 1:
                stage = data[:32].split(b'\0',1)[0].decode('ascii')
                ns, begin, deadline, result = struct.unpack_from('<4q',data,32)
                cycle, generation, io, joint, requested, length, captured = struct.unpack_from('<7Q',data,64)
                position, feedback = struct.unpack_from('<2d',data,120)
                flags, error, ident = struct.unpack_from('<iiI',data,136)
                if not stage or captured > 64:
                    raise ValueError('invalid command snapshot record')
                yield dict(seq=seq,stage=stage,ns=ns,begin_ns=begin,cycle=cycle,joint=joint,
                    generation=generation,io=io,position=position if math.isfinite(position) else None,
                    feedback=feedback if math.isfinite(feedback) else None,flags=flags,deadline=deadline,
                    result=result,errno=error,requested=requested,length=length,id=ident,
                    captured=captured,hex=data[148:148+captured].hex())
            else:
                begin, end, size, captured = struct.unpack_from('<qqQQ',data,0)
                result, tx = data[32:34]
                if captured > 1024 or tx not in (0,1):
                    raise ValueError('invalid raw snapshot record')
                yield dict(sequence=seq,direction='tx' if tx else 'rx',begin_ns=begin,end_ns=end,
                           result=result,size=size,captured=captured,hex=data[34:34+captured].hex())


def export_snapshot(source, destination, limit, kind):
    meta = snapshot_metadata(source, kind)
    header = {key:meta[key] for key in ('schema_version','capacity','total')}
    if kind == 1:
        header.update(clock='steady_clock_ns', dropped=meta['dropped'], closed=meta['closed'])
    else:
        header.update(overwritten=meta['overwritten'], closed=meta['closed'])
    output = LimitedFile(destination, limit)
    try:
        with gzip.GzipFile(filename='', mode='wb', fileobj=output, mtime=0, compresslevel=1) as compressed:
            compressed.write((json.dumps(header,separators=(',',':'))+'\n').encode())
            for row in snapshot_rows(source, meta):
                compressed.write((json.dumps(row,separators=(',',':'),allow_nan=False)+'\n').encode())
    finally:
        output.close()
    return meta


def finalize(snapshot, run, session_root, diagnostics=None, confirmed_exit=False):
    """Caller guarantees serial owner exit and a completed/attempted disable helper."""
    snapshot, run, session_root = Path(snapshot), Path(run), Path(session_root)
    owner = json.loads((snapshot/'owner.json').read_text())
    if Path(owner['run']).resolve() != run.resolve() or owner['uid'] != os.getuid():
        raise ValueError('snapshot ownership mismatch; refusing cleanup/export')
    if not confirmed_exit:
        require_writer_exited(owner)
    report = dict(snapshot_dir=str(snapshot), diagnostic_status='incomplete', snapshots={},
                  quotas=dict(session_bytes=SESSION_LIMIT,root_bytes=ROOT_LIMIT,free_reserve=FREE_RESERVE),
                  diagnostics=diagnostics or {})
    errors = []; export_failed = False
    for filename, output_name, kind, cap in (
            ('chain.snapshot','command-chain.jsonl.gz',1,96*MIB),
            ('serial.snapshot','serial-trace.jsonl.gz',2,16*MIB)):
        try:
            output_path = run/output_name
            if output_path.exists():
                # A failed/crashed export remains evidence; create a successor.
                output_path = run/(output_name.removesuffix('.jsonl.gz')+'.recovered-'+uuid.uuid4().hex[:8]+'.jsonl.gz')
            remaining = min(cap, SESSION_LIMIT-directory_bytes(session_root)-10*MIB,
                            ROOT_LIMIT-directory_bytes(session_root.parent)-10*MIB,
                            shutil.disk_usage(run).free-FREE_RESERVE)
            if remaining <= 0:
                raise OSError('persistent log quota/free-space exhausted')
            meta = export_snapshot(snapshot/filename,output_path,remaining,kind)
            meta['output_file'] = output_path.name
            report['snapshots'][filename] = meta
            if not meta['closed'] or meta['dropped'] or meta['overwritten']:
                errors.append(filename+': incomplete capture')
        except (OSError, ValueError, UnicodeError, struct.error) as exc:
            export_failed = True
            errors.append(filename+': '+str(exc))
    if any((diagnostics or {}).get(key) for key in ('events_dropped','events_error','framework_discarded','framework_error')):
        errors.append('operator/framework logs incomplete')
    report['errors'] = errors
    report['diagnostic_status'] = 'complete' if not errors else 'incomplete'
    report['snapshot_retained'] = export_failed
    # Failure to persist the outcome must retain evidence and block restart.
    manifest_path = run/'capture-manifest.json'
    if manifest_path.exists():
        # Preserve previous outcome on recovery, then publish the new index.
        previous = run/('capture-manifest.previous-'+uuid.uuid4().hex[:8]+'.json')
        previous.write_bytes(manifest_path.read_bytes())
    manifest_path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    if not export_failed:
        # Only this exact unique directory, after verified owner metadata and export.
        shutil.rmtree(snapshot)
    return report


def main():
    parser = argparse.ArgumentParser(description='离线恢复已退出框架的快照；不访问电机。保留失败快照及既有日志。')
    parser.add_argument('--snapshot-dir', required=True, type=Path)
    parser.add_argument('--run-dir', required=True, type=Path)
    parser.add_argument('--session-root', required=True, type=Path)
    args = parser.parse_args()
    report = finalize(args.snapshot_dir,args.run_dir,args.session_root)
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['diagnostic_status'] == 'complete' else 1


if __name__ == '__main__':
    raise SystemExit(main())
