import hashlib
import json
from pathlib import Path
import sys

from speed_planning import plan
from speed_policy import PROFILE, configure_simulation_profile


ROOT = Path(__file__).resolve().parent
SOURCE_NAMES = ('centered_exit.py', 'table_alignment.py', 'axial_transport.py', 'axial_pilot.py', 'bounded_release_alignment.py', 'release_alignment.py', 'speed_worker.py', 'speed_policy.py', 'sequential_worker.py', 'grip_limits.py',
                'speed_planning.py', 'speed_retrieval_geometry.py', 'sequential_geometry.py', 'sequential_planning.py')


def require_profile(root):
    identity = json.loads((root / 'identity.json').read_text())
    if identity.get('actual_runtime_subclass') != 'SpeedPriorityCycle' or identity.get('speed_profile') != PROFILE:
        raise ValueError('Explicit sealed speed-priority identity required')
    for name in SOURCE_NAMES:
        if hashlib.sha256((root / name).read_bytes()).hexdigest() != identity['sha256'][name]:
            raise ValueError('Sealed worker source changed: ' + name)


def main():
    if len(sys.argv) != 3:
        raise ValueError('Actual snapshot and never-used result paths required')
    require_profile(ROOT)
    configure_simulation_profile()
    source, destination = map(Path, sys.argv[1:])
    if destination.exists() or destination.is_symlink():
        raise FileExistsError(str(destination))
    content = source.read_bytes()
    result = plan(json.loads(content))
    result.update(source_snapshot_sha256=hashlib.sha256(content).hexdigest(),
                  worker_source_sha256=hashlib.sha256((ROOT / 'sequential_worker.py').read_bytes()).hexdigest(),
                  speed_worker_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  speed_planning_source_sha256=hashlib.sha256((ROOT / 'speed_planning.py').read_bytes()).hexdigest(),
                  speed_retrieval_geometry_source_sha256=hashlib.sha256((ROOT / 'speed_retrieval_geometry.py').read_bytes()).hexdigest(),
                  axial_pilot_source_sha256=hashlib.sha256((ROOT / 'axial_pilot.py').read_bytes()).hexdigest(),
                  axial_transport_source_sha256=hashlib.sha256((ROOT / 'axial_transport.py').read_bytes()).hexdigest(),
                  centered_exit_source_sha256=hashlib.sha256((ROOT / 'centered_exit.py').read_bytes()).hexdigest(),
                  table_alignment_source_sha256=hashlib.sha256((ROOT / 'table_alignment.py').read_bytes()).hexdigest(),
                  speed_profile=PROFILE, original_strict_holder_certificate_not_claimed=True,
                  geometric_hints_sha256=hashlib.sha256((ROOT / 'inputs/sequential_hints.json').read_bytes()).hexdigest())
    with destination.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print('SPEED_SEQUENTIAL_GEOMETRY_ONLY_NOT_PHYSICAL', result['mode'], result['checked_samples'], flush=True)


if __name__ == '__main__':
    main()
