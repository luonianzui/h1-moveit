import hashlib
import importlib.util
import json
import os
from pathlib import Path


ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[1]
RUN = WORKSPACE / 'runtime_evidence/centrifuge_48box_sequential_full_20261022_r351'


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


specification = importlib.util.spec_from_file_location('bounded_cache_cleanup', ROOT / 'cache_cleanup.py')
module = importlib.util.module_from_spec(specification)
specification.loader.exec_module(module)
inventory = json.loads((ROOT / 'inventory.json').read_text())
cleanup = json.loads((ROOT / 'cleanup.json').read_text())
journal = [json.loads(line) for line in (ROOT / 'deletion_journal.jsonl').read_text().splitlines()]
assert cleanup['status'] == 'PASS' and len(journal) == len(inventory['members']) == 1546
protected = module.protected_and_installed()
for original, deleted in zip(inventory['members'], journal):
    path = Path(original['path'])
    assert deleted == {key: original[key] for key in ('path', 'bytes', 'sha256')}
    assert not path.exists() and not path.is_symlink()
    assert str(path) not in protected
    assert original['kind'] == 'cmake_compile_intermediate'
    recipe = original['recipe']
    assert digest(Path(recipe['source'])) == recipe['source_sha256']
    assert digest(Path(recipe['build_make'])) == recipe['build_make_sha256']
bindings = json.loads((ROOT / 'before_protection_bindings.json').read_text())
protection = {}
assert len(bindings) == 9
for name, expected in bindings.items():
    before = Path(name)
    assert digest(before) == expected
    after = before.with_name(before.name.replace('_before.json', '_after.json'))
    assert json.loads(before.read_text()) == json.loads(after.read_text())
    assert json.loads(after.read_text())['status'] == 'PASS'
    assert after.stat().st_mtime_ns > (ROOT / 'cleanup.json').stat().st_mtime_ns
    protection[str(after)] = digest(after)
identity = json.loads((RUN / 'identity.json').read_text())
for name, expected in identity['sha256'].items():
    assert digest(RUN / name) == expected, name
result = dict(status='PASS_BOUNDED_INTERMEDIATE_CACHE_CLEANUP',
    removed_count=len(journal), removed_logical_bytes=sum(row['bytes'] for row in journal),
    removed_allocated_bytes=cleanup['removed_allocated_bytes'],
    all_listed_files_absent=True, current_protected_paths_disjoint=True,
    retained_sources_and_build_recipes_hash_unchanged=True,
    nine_post_cleanup_protection_groups_same_pass=protection,
    run_source_members_unchanged=len(identity['sha256']),
    run_source_identity_sha256=digest(RUN / 'identity.json'),
    current_free_bytes=os.statvfs(WORKSPACE).f_bavail * os.statvfs(WORKSPACE).f_frsize,
    full_run_startup_budget_bytes=2096208487,
    process_visibility_limits_retained_in_original_logs=True,
    free_space_delta_not_identical_to_removed_allocated_bytes=True,
    next_compilation_rebuilds_removed_intermediates=True)
with (ROOT / 'after_verification.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result), flush=True)
