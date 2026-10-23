import json
import math
from pathlib import Path
import re
import signal
import subprocess
import threading
import time

from camera_guard import restore_with_retries, should_restore


def pose_values(message):
    position = message['position']
    orientation = message['orientation']
    values = [float(position.get(key, 0.)) for key in ('x', 'y', 'z')]
    quaternion = [float(orientation.get(key, 0.)) for key in ('x', 'y', 'z', 'w')]
    if not all(math.isfinite(value) for value in values + quaternion):
        raise ValueError('Nonfinite native GUI camera pose')
    norm = math.sqrt(sum(value * value for value in quaternion))
    if abs(norm - 1.) > .001:
        raise ValueError('Invalid native GUI camera quaternion')
    return values, [value / norm for value in quaternion]


def requested_pose(request):
    groups = re.findall(r'(position|orientation)\s*:\s*\{([^}]+)\}', request)
    message = {name: {key: float(value) for key, value in re.findall(
        r'([xyzw])\s*:\s*([-+0-9.eE]+)', contents)} for name, contents in groups}
    assert set(message) == {'position', 'orientation'}
    assert set(message['position']) == {'x', 'y', 'z'}
    assert set(message['orientation']) == {'x', 'y', 'z', 'w'}
    pose_values(message)
    return message


def pose_check(snapshot, target, now):
    if snapshot is None:
        return dict(valid=False, reason='no_native_camera_pose')
    age = now - snapshot['received_monotonic']
    if not math.isfinite(age) or not 0. <= age <= 1.:
        return dict(valid=False, reason='stale_native_camera_pose', age_wall_s=age)
    position, quaternion = pose_values(snapshot['pose'])
    goal_position, goal_quaternion = pose_values(target)
    distance = math.dist(position, goal_position)
    dot = abs(sum(first * second for first, second in zip(quaternion, goal_quaternion)))
    angle = math.degrees(2. * math.acos(min(1., dot)))
    return dict(valid=True, matches=distance <= .002 and angle <= .1,
                position_error_m=distance, orientation_error_deg=angle,
                age_wall_s=age, sequence=snapshot['sequence'])


class PoseReader:
    def __init__(self, folder):
        self.folder = Path(folder)
        self.latest = None
        self.fault = None
        self.sequence = 0
        self.lock = threading.Lock()
        self.stderr = (self.folder / 'camera_pose_transport.log').open('x')
        self.journal = (self.folder / 'camera_pose_native.jsonl').open('x')
        self.process = subprocess.Popen(['gz', 'topic', '-e', '-t', '/gui/camera/pose', '--json-output'],
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=self.stderr, text=True, bufsize=1)
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()

    def read(self):
        buffer = ''
        decoder = json.JSONDecoder()
        try:
            for line in self.process.stdout:
                buffer += line
                if len(buffer) > 65536:
                    raise ValueError('Native camera JSON exceeds bounded buffer')
                buffer = buffer.lstrip()
                while buffer:
                    try:
                        message, end = decoder.raw_decode(buffer)
                    except json.JSONDecodeError:
                        break
                    buffer = buffer[end:].lstrip()
                    pose_values(message)
                    self.sequence += 1
                    row = dict(pose=message, received_monotonic=time.monotonic(),
                               wall=time.time(), sequence=self.sequence)
                    self.journal.write(json.dumps(row) + '\n')
                    self.journal.flush()
                    with self.lock:
                        self.latest = row
            if buffer:
                raise ValueError('Truncated native camera JSON')
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.fault = repr(error)

    def snapshot(self):
        with self.lock:
            return self.latest

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            self.process.wait(timeout=5)
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            raise RuntimeError('Native camera reader did not exit')
        self.process.stdout.close()
        self.stderr.close()
        self.journal.close()


def verified_restore(request, phase, reader, target, stop_event, restore=restore_with_retries,
                     clock=time.monotonic):
    before = reader.snapshot()
    check = pose_check(before, target, clock())
    if check['valid'] and check['matches']:
        return dict(phase=phase, wall=time.time(), status='PASS', action='native_pose_already_correct',
                    pose=check, service_rows=[])
    if not check['valid']:
        raise RuntimeError('Camera pose cannot be verified: ' + check['reason'])
    accepted, rows, error = restore(request, phase)
    deadline = clock() + 3.
    while not stop_event.is_set() and clock() < deadline:
        current = reader.snapshot()
        check = pose_check(current, target, clock())
        if current is not None and current['sequence'] > before['sequence'] and check['valid'] and check['matches']:
            return dict(phase=phase, wall=time.time(), status='PASS', action='restored_and_native_pose_verified',
                        pose=check, service_rows=rows, service_accepted=accepted, service_error=error)
        stop_event.wait(.05)
    if stop_event.is_set():
        return dict(phase=phase, wall=time.time(), status='STOPPED', action='normal_shutdown',
                    pose=check, service_rows=rows)
    raise RuntimeError('GUI restore lacks a fresh matching native camera pose')


class CameraGuard:
    def __init__(self, folder):
        self.folder = Path(folder)
        self.request = json.loads((self.folder / 'camera_request.json').read_text())['request']
        self.target = requested_pose(self.request)
        self.stop_event = threading.Event()
        self.rows = []
        self.fault = None
        self.reader = None
        self.thread = threading.Thread(target=self.run, daemon=True)

    def start(self):
        self.thread.start()

    def run(self):
        try:
            self.reader = PoseReader(self.folder)
            deadline = time.monotonic() + 5.
            while self.reader.snapshot() is None and time.monotonic() < deadline:
                if self.stop_event.wait(.05):
                    return
                if self.reader.fault is not None:
                    raise RuntimeError(self.reader.fault)
            phase, checked = None, 0.
            while not self.stop_event.wait(.2):
                if (self.folder / 'result.json').is_file():
                    continue
                if self.reader.fault is not None or self.reader.process.poll() is not None:
                    raise RuntimeError('Native camera transport failed: ' + str(self.reader.fault))
                progress = self.folder / 'trace_progress.json'
                current = json.loads(progress.read_text()).get('phase') if progress.is_file() else 'preflow'
                if should_restore(phase, current, time.monotonic() - checked, False):
                    row = verified_restore(self.request, current, self.reader, self.target, self.stop_event)
                    self.rows.append(row)
                    phase, checked = current, time.monotonic()
        except (OSError, ValueError, RuntimeError, KeyError, TypeError, subprocess.SubprocessError) as error:
            self.fault = repr(error)

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=22)
        if self.thread.is_alive():
            self.fault = 'Camera verification did not finish bounded retries'
        if self.reader is not None and not self.thread.is_alive():
            try:
                self.reader.close()
            except (OSError, RuntimeError, subprocess.SubprocessError) as error:
                self.fault = repr(error)
        with (self.folder / 'camera_guard.json').open('x') as stream:
            json.dump(dict(status='PASS' if self.fault is None else 'FAIL', error=self.fault, rows=self.rows,
                native_camera_pose_topic='/gui/camera/pose', position_tolerance_m=.002,
                orientation_tolerance_deg=.1, pose_max_age_wall_s=1.,
                redundant_pose_service_requests_skipped=True, acknowledgement_alone_never_proves_camera_pose=True,
                terminal_result_stops_restoration_for_supervisor_overhead_movie=True,
                service_only_changes_gui_camera_not_physical_models=True), stream, indent=2)
