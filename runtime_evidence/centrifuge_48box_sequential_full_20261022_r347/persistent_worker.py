import hashlib
import json
import os
from pathlib import Path
import sys
import time


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def request_paths(root, message, token, sequence):
    if message.get('token') != token or message.get('sequence') != sequence:
        raise ValueError('Worker token or sequence mismatch')
    source, destination = Path(message['source']), Path(message['destination'])
    if any(path.parent != root / 'run' or path.is_symlink() for path in (source, destination)):
        raise ValueError('Only this run exact non-symlink paths are allowed')
    if not source.name.endswith('_request.json') or destination.name != source.name.replace('_request.json', '_result.json'):
        raise ValueError('Mismatched actual planning request and result')
    receipt = destination.with_suffix('.complete.json')
    if not source.is_file() or destination.exists() or receipt.exists():
        raise FileExistsError('Missing request or previously used output')
    return source, destination, receipt


def serve(root, token, handle, incoming):
    with (root / 'run/planner_ready.json').open('x') as stream:
        json.dump(dict(token=token, pid=os.getpid()), stream)
    for sequence, line in enumerate(incoming, 1):
        message = json.loads(line)
        source, destination, receipt = request_paths(root, message, token, sequence)
        source_hash = digest(source)
        began = time.monotonic()
        handle(source, destination)
        if digest(source) != source_hash:
            raise RuntimeError('Actual planning snapshot changed during planning')
        with receipt.open('x') as stream:
            json.dump(dict(token=token, sequence=sequence, pid=os.getpid(),
                source_sha256=source_hash, result_sha256=digest(destination),
                planning_wall_s=time.monotonic()-began), stream)


def main():
    from speed_worker import main as original_worker_main, require_profile
    root = Path(__file__).resolve().parent
    require_profile(root)
    identity = json.loads((root / 'identity.json').read_text())
    for name in ('persistent_worker.py', 'planner_client.py', 'planning_pipeline.py', 'joined_stages.py'):
        if digest(root / name) != identity['sha256'][name]:
            raise RuntimeError('Sealed persistent planner source changed: ' + name)
    token = sys.argv[1]

    def handle(source, destination):
        previous = sys.argv
        try:
            sys.argv = [str(root / 'speed_worker.py'), str(source), str(destination)]
            original_worker_main()
        finally:
            sys.argv = previous

    serve(root, token, handle, sys.stdin)


if __name__ == '__main__':
    main()
