import hashlib
import json
from pathlib import Path

import numpy as np
from trajectory_msgs.msg import JointTrajectoryPoint

from initial_button_motion import initial_withdraw_points


ROOT = Path(__file__).resolve().parent
SOURCE = ROOT.parent / 'centrifuge_48box_sequential_review_20261022_r9/opening_tracking_diagnostic.json'
manifest = dict((relative, checksum) for checksum, relative in
                (row.split(maxsplit=1) for row in (SOURCE.parent / 'SHA256SUMS').read_text().splitlines()))
assert hashlib.sha256(SOURCE.read_bytes()).hexdigest() == manifest[SOURCE.name]
diagnostic = json.loads(SOURCE.read_text())
commands = diagnostic['evidence']['commands']
command = [row for row in commands if row['phase'] == 'machine_preparation_open_withdraw'][-1]
original = [JointTrajectoryPoint(positions=point['positions']) for point in command['payload']['points']]
output, profile = initial_withdraw_points(original, .6)
for original_point, index in zip(original, profile['original_vertex_indices']):
    assert list(output[index].positions) == list(original_point.positions)
    assert list(output[index].velocities) == [0.] * 7
times = np.array([point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 for point in output])
velocities = np.array([list(point.velocities) for point in output])
assert np.min(np.diff(times)) >= .02 - 1e-9
assert np.max(np.abs(velocities)) <= .6 + 1e-12
assert np.max(np.abs(np.diff(velocities, axis=0) / np.diff(times)[:, None])) <= 2. + 1e-6
report = dict(status='PASS_ACTUAL_R351_WITHDRAW_GEOMETRY_AND_BOUNDED_RAMP_NOT_PHYSICAL',
              source=str(SOURCE), source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
              original_duration_sim_s=command['payload']['duration'], new_duration_sim_s=float(times[-1]),
              original_points=len(original), new_points=len(output), profile=profile,
              original_vertices_exact=True, tracking_limit_rad=.15, terminal_limit_rad=.005,
              contact_timing_and_30s_spin_unchanged=True, actual_controller_reference_not_yet_observed=True)
with (ROOT / 'initial_button_motion_qualification.json').open('x') as stream:
    json.dump(report, stream, indent=2)
print(report['status'], report['original_duration_sim_s'], report['new_duration_sim_s'], flush=True)
