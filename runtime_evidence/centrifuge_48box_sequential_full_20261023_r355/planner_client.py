import hashlib
import json
from pathlib import Path
import signal
import subprocess
import time
import uuid


class PlannerClient:
    def __init__(self, root):
        self.root = Path(root)
        self.token = uuid.uuid4().hex
        self.sequence = 0
        self.receipts = []
        self.pending = None
        self.log = (self.root / 'run/persistent_planner.log').open('x')
        self.process = subprocess.Popen(['/usr/bin/python3', str(self.root / 'persistent_worker.py'), self.token],
            stdin=subprocess.PIPE, stdout=self.log, stderr=subprocess.STDOUT, text=True, start_new_session=True)

    def execute(self, source, destination, spin):
        return self.finish(self.submit(source, destination), spin)

    def submit(self, source, destination):
        if self.pending is not None:
            raise RuntimeError('Only one unconsumed planning request is permitted')
        receipt_path = destination.with_suffix('.complete.json')
        if destination.exists() or receipt_path.exists():
            raise FileExistsError(str(destination))
        self.sequence += 1
        message = dict(token=self.token, sequence=self.sequence, source=str(source), destination=str(destination))
        self.process.stdin.write(json.dumps(message) + '\n')
        self.process.stdin.flush()
        self.pending = dict(source=source, destination=destination, receipt_path=receipt_path,
            sequence=self.sequence, deadline=time.monotonic() + 120)
        return self.pending

    def finish(self, ticket, spin):
        if ticket is not self.pending or ticket is None:
            raise RuntimeError('Unknown or previously consumed planning ticket')
        source, destination = ticket['source'], ticket['destination']
        receipt_path = ticket['receipt_path']
        while True:
            if self.process.poll() is not None:
                raise RuntimeError('Persistent planner exited before request completion')
            if receipt_path.exists():
                try:
                    receipt = json.loads(receipt_path.read_text())
                except json.JSONDecodeError:
                    receipt = None
                if receipt is not None:
                    expected = dict(token=self.token, sequence=ticket['sequence'], pid=self.process.pid,
                        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                        result_sha256=hashlib.sha256(destination.read_bytes()).hexdigest())
                    if any(receipt.get(key) != value for key, value in expected.items()):
                        raise RuntimeError('Persistent planner completion identity mismatch')
                    self.receipts.append(receipt)
                    self.pending = None
                    return receipt
            if time.monotonic() >= ticket['deadline']:
                raise TimeoutError('Persistent planner request timeout; no motion dispatched')
            spin()

    def close(self):
        self.process.stdin.close()
        try:
            self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.process.send_signal(signal.SIGINT)
            self.process.wait(timeout=10)
        finally:
            self.log.close()
        with (self.root / 'run/planner_lifecycle.json').open('x') as stream:
            json.dump(dict(pid=self.process.pid, returncode=self.process.returncode,
                process_launches=1, requests=self.sequence, completed=self.receipts,
                fresh_scene_per_request=True, cached_plans_or_holder_anchors=False), stream, indent=2)
