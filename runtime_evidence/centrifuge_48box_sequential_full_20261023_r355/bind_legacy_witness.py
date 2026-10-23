import copy
import hashlib
import json
from pathlib import Path

from inherited_source_identity import restore_inherited_source


ROOT = Path(__file__).resolve().parent
VERIFIED = ROOT.parent / 'centrifuge_48box_elbow_full_20261016_r291'


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def rebind(content):
    return content.replace(str(VERIFIED), str(ROOT)).replace(VERIFIED.name, ROOT.name).replace(
        'install_lab_scene_48box_elbow_full_20261016_r291', 'install_lab_scene_48box_sequential_full_20261023_r355').replace(
        'centrifuge_48box_elbow_full_r291', 'centrifuge_48box_sequential_full_r349').replace('h1_elbow_r291', 'h1_seq_full_r337')


manifest = {relative.lstrip('*'): expected for expected, relative in
            (line.split(maxsplit=1) for line in (VERIFIED / 'SHA256SUMS').read_text().splitlines())}
assert digest(VERIFIED / 'derivation.json') == manifest['derivation.json']
old = json.loads((VERIFIED / 'derivation.json').read_text())
counts = {}
for name in ('centrifuge_48box_bucket_axis_exit_full_20261015_r282', 'centrifuge_48box_startup_ramp_probe_20261014_r1'):
    folder = ROOT.parent / name
    rows = (folder / 'SHA256SUMS').read_text().splitlines()
    for line in rows:
        expected, relative = line.split(maxsplit=1)
        assert digest(folder / relative.lstrip('*')) == expected
    counts[name] = len(rows)
assert sorted(counts.values()) == [51, 563]
records = []
for original in old['records']:
    name = original['relative']
    if not (ROOT / name).is_file():
        continue
    assert digest(VERIFIED / name) == manifest[name], name
    data = (VERIFIED / name).read_bytes()
    if (VERIFIED / name).suffix in ('.py', '.sh', '.json', '.sdf', '.urdf', '.yaml', '.md', '.config') and name != 'offline_profile_revision1_source_snapshot.json':
        content = rebind(data.decode())
        if (VERIFIED / name).suffix == '.py' and '/' not in name:
            content = restore_inherited_source(name, content)
        data = content.encode()
    row = copy.deepcopy(original)
    row.update(derived_sha256=hashlib.sha256(data).hexdigest(),
               independently_bound_r291_sealed_sha256=manifest[name],
               witness_scope='LEGACY_INVERSE_SOURCE_CHECK_NOT_SEQUENTIAL_EXECUTION_CERTIFICATE')
    records.append(row)
document = dict(old, records=records, probe_frozen_members_verified=51, parent_full_frozen_members_verified=563,
                witness_parent=str(VERIFIED), witness_parent_manifest_sha256=digest(VERIFIED / 'SHA256SUMS'),
                current_runtime_separately_declared_in='runtime_derivation.json', new_physical_success_claimed=False)
with (ROOT / 'derivation.json').open('x') as stream:
    json.dump(document, stream, indent=2)
print('LEGACY_WITNESS_REBOUND_FROM_VERIFIED_PARENTS', len(records), counts, flush=True)
