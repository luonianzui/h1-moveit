import json
from pathlib import Path
import subprocess
import threading
import time


def should_restore(last_phase, phase, elapsed_wall_s, terminal_result):
    return not terminal_result and (last_phase != phase or elapsed_wall_s >= 4.)


class CameraGuard:
    def __init__(self, folder):
        self.folder = Path(folder)
        self.request = json.loads((self.folder / 'camera_request.json').read_text())['request']
        self.stop_event = threading.Event()
        self.rows = []
        self.fault = None
        self.thread = threading.Thread(target=self.run, daemon=True)

    def start(self):
        self.thread.start()

    def run(self):
        phase, restored = None, 0.
        while not self.stop_event.wait(.2):
            try:
                terminal = (self.folder / 'result.json').is_file()
                progress = self.folder / 'trace_progress.json'
                current = json.loads(progress.read_text()).get('phase') if progress.is_file() else 'preflow'
                now = time.monotonic()
                if not should_restore(phase, current, now - restored, terminal):
                    continue
                response = subprocess.run(['gz', 'service', '-s', '/gui/move_to/pose', '--reqtype', 'gz.msgs.GUICamera',
                    '--reptype', 'gz.msgs.Boolean', '--timeout', '1200', '--req', self.request],
                    stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=3)
                accepted = response.returncode == 0 and 'data: true' in response.stdout
                self.rows.append(dict(wall=time.time(), phase=current, returncode=response.returncode,
                                     accepted=accepted, output=response.stdout, error=response.stderr))
                if not accepted:
                    self.fault = 'Wide scene camera request not acknowledged'
                    return
                phase, restored = current, time.monotonic()
            except (OSError, ValueError, subprocess.TimeoutExpired) as error:
                self.fault = str(error)
                return

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=4)
        if self.thread.is_alive():
            self.fault = 'Camera guard did not finish bounded service request'
        with (self.folder / 'camera_guard.json').open('x') as stream:
            json.dump(dict(status='PASS' if self.fault is None else 'FAIL', error=self.fault,
                           rows=self.rows, service_only_changes_gui_camera_not_physical_models=True,
                           terminal_result_stops_restoration_for_supervisor_overhead_movie=True,
                           periodic_four_wall_seconds_and_phase_change=True), stream, indent=2)
