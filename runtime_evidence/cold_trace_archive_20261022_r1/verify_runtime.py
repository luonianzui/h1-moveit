import builtins
import hashlib
import io
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[1]
SOURCE = ROOT.parent / 'centrifuge_48box_sequential_full_20261022_r351'
TARGETS = {
    'centrifuge_48box_servo_gain02_20261007_r144',
    'centrifuge_48box_velocity_epsilon_20261007_r147',
    'centrifuge_48box_passive_rotor_settle_20261007_r148',
    'centrifuge_48box_drop_contact_settle_20261007_r149',
    'centrifuge_48box_left_reorient_20261008_r150',
}
phase = sys.argv[1]
assert phase in ('before', 'after')
identity = json.loads((SOURCE / 'identity.json').read_text())['sha256']
opened_targets = []
original_open, original_io_open = builtins.open, io.open
sys.path.insert(0, str(SOURCE))


def track(opener, path, *args, **kwargs):
    if not isinstance(path, int):
        candidate = Path(path).absolute()
        if candidate.name == 'loading_trace.jsonl' and candidate.parent.parent.name in TARGETS:
            opened_targets.append(str(candidate))
    return opener(path, *args, **kwargs)


builtins.open = lambda path, *args, **kwargs: track(original_open, path, *args, **kwargs)
io.open = lambda path, *args, **kwargs: track(original_io_open, path, *args, **kwargs)
try:
    for name in ('verify_dual_protection.py', 'verify_current_protection.py', 'verify_startup_protection.py'):
        source = SOURCE / name
        assert hashlib.sha256(source.read_bytes()).hexdigest() == identity[name]
        sys.argv = [str(ROOT / name), phase]
        exec(compile(source.read_text(), str(source), 'exec'),
             dict(__name__='__main__', __file__=str(ROOT / name)))
finally:
    builtins.open, io.open = original_open, original_io_open
assert not opened_targets, opened_targets
groups = {}
for prefix in ('protected', 'source_1009', 'recent_protected', 'latest_protected',
               'resumed_protected', 'additional_protected', 'dual_protected',
               'current_protected', 'startup_protected'):
    result = json.loads((ROOT / (prefix + '_' + phase + '.json')).read_text())
    assert result['status'] == 'PASS'
    groups[prefix] = hashlib.sha256((ROOT / (prefix + '_' + phase + '.json')).read_bytes()).hexdigest()
    if phase == 'after':
        assert result == json.loads((ROOT / (prefix + '_before.json')).read_text())
with (ROOT / ('runtime_protection_' + phase + '.json')).open('x') as stream:
    json.dump(dict(status='PASS', unchanged_original_protection_code=True,
        ordinary_existing_archive_wrapper=True, cold_target_reads=opened_targets,
        groups=groups), stream, indent=2)
print('CURRENT_RUNTIME_NINE_GROUPS_PASS_NO_COLD_LOG_READS', phase, flush=True)
