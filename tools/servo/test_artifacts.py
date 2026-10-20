"""Bounded, allowlisted diagnostics for synthetic PTY tests (no device access)."""
import gzip
import json
from pathlib import Path


def retain(source, destination, limit=256 * 1024 * 1024):
    """Keep at most limit payload bytes; explicitly report partial captures.

    Unsealed snapshots can be large sparse files. Compress them while copying;
    never copy an entire arbitrary temporary tree or follow a link outside it.
    """
    source, destination = Path(source), Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    report = {'complete': True, 'files': [], 'limit_bytes': limit}
    remaining = limit
    for path in sorted(source.rglob('*')):
        if path.is_symlink() or not path.is_file():
            continue
        if path.suffix not in ('.json', '.jsonl', '.gz', '.log', '.txt', '.csv', '.urdf', '.yaml', '.snapshot'):
            continue
        relative = path.relative_to(source)
        target = destination / relative
        if path.suffix == '.snapshot':
            target = target.with_suffix('.snapshot.gz')
        target.parent.mkdir(parents=True, exist_ok=True)
        complete = False
        if remaining < 131072:
            report['complete'] = False
            report['files'].append({'path': relative.as_posix(), 'complete': False,
                                    'source_bytes': path.stat().st_size, 'retained_bytes': 0})
            continue
        with path.open('rb') as incoming, target.open('wb') as raw:
            stream = gzip.GzipFile(fileobj=raw, mode='wb') if path.suffix == '.snapshot' else raw
            try:
                # Bound both retained size and work spent on malformed captures.
                consumed = 0
                # One chunk plus compression/header/footer overhead must fit.
                while remaining - raw.tell() >= 131072 and consumed < 2 * 1024**3:
                    chunk = incoming.read(65536)
                    if not chunk:
                        complete = True
                        break
                    stream.write(chunk)
                    if stream is not raw:
                        stream.flush()
                    consumed += len(chunk)
            finally:
                if stream is not raw:
                    stream.close()
            remaining -= raw.tell()
        report['complete'] &= complete
        report['files'].append({'path': relative.as_posix(), 'complete': complete,
                                'source_bytes': path.stat().st_size,
                                'retained_bytes': target.stat().st_size})
    (destination / 'artifact-manifest.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    return report
