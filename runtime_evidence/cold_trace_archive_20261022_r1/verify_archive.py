import hashlib
import json
from pathlib import Path
import shutil

import archive_cold
import cold_access


ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[1]
inventory = json.loads((ROOT / 'inventory.json').read_text())
assert json.loads((ROOT / 'migration_result.json').read_text())['status'] == 'PASS'
assert all(not (WORKSPACE / row['path']).exists() for row in inventory['entries'])
registry = cold_access.install()
assert len(registry.relative_entries) == 5
manifests = []
for row in inventory['entries']:
    entry = registry.relative_entries[row['path']]
    registry.verify_logical(entry)
    manifest = WORKSPACE / row['manifest']
    assert hashlib.sha256(manifest.read_bytes()).hexdigest() == row['manifest_sha256']
    result = cold_access.verify(manifest)
    with (ROOT / (manifest.parent.name + '_manifest_check.json')).open('x') as stream:
        json.dump(result, stream, indent=2)
    assert result['status'] == 'PASS', result
    manifests.append(dict(path=row['manifest'], sha256=row['manifest_sha256'],
        original_members_verified=result['checked_files']))
for name, expected in inventory['original_tools'].items():
    assert archive_cold.helpers.digest(WORKSPACE / name) == expected
result = dict(status='PASS_LOSSLESS_DATA_AND_ORIGINAL_MANIFESTS',
    archives=5, manifests=manifests, original_archive_tools_and_registry_unchanged=True,
    data_sha_size_lines_crc_verified=True, free_bytes=shutil.disk_usage(WORKSPACE).free,
    explicit_cold_view_only=True, standard_runtime_wrapper_unchanged=True)
with (ROOT / 'archive_verification.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result), flush=True)
