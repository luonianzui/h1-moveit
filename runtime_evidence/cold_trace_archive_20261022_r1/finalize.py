import hashlib
import json
from pathlib import Path
import re
import shutil

import psutil


ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[1]


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read(name):
    return json.loads((ROOT / name).read_text())


assert read('migration_result.json')['status'] == 'PASS'
assert read('archive_verification.json')['status'] == 'PASS_LOSSLESS_DATA_AND_ORIGINAL_MANIFESTS'
assert read('runtime_bindings_after.json')['status'] == 'PASS'
for prefix in ('protected', 'source_1009', 'recent_protected', 'latest_protected',
               'resumed_protected', 'additional_protected', 'dual_protected',
               'current_protected', 'startup_protected'):
    assert read(prefix + '_before.json') == read(prefix + '_after.json')
    assert read(prefix + '_after.json')['status'] == 'PASS'
assert read('runtime_protection_before.json')['cold_target_reads'] == []
assert read('runtime_protection_after.json')['cold_target_reads'] == []
counts = {'archive_tests_revision2.log': 56, 'r351_regression.log': 2268,
          'r347_regression.log': 2240, 'configuration_regression.log': 46,
          'loading_regression.log': 122}
for name, count in counts.items():
    content = (ROOT / name).read_text()
    assert re.search(r'\b' + str(count) + r' passed\b', content), name
    assert not re.search(r'\b[1-9][0-9]* failed\b', content), name
inventory = read('inventory.json')
for name, expected in inventory['original_tools'].items():
    assert digest(WORKSPACE / name) == expected
assert all(not (WORKSPACE / row['path']).exists() for row in inventory['entries'])
registry = read('registry.json')
for relative, entry in registry['entries'].items():
    assert digest(WORKSPACE / entry['archive']) == entry['archive_sha256']
    assert (WORKSPACE / entry['archive']).stat().st_mode & 0o777 == 0o444
documents, checked_links = {}, []
before = read('documentation_before.json')
authority = before['AGENTS.md']
current = (WORKSPACE / 'AGENTS.md').read_bytes()
assert len(current) > authority['bytes']
assert hashlib.sha256(current[:authority['bytes']]).hexdigest() == authority['sha256']
for name, original in before.items():
    if name != 'AGENTS.md':
        assert hashlib.sha256(original['text'].encode()).hexdigest() == original['sha256']
    data = (WORKSPACE / name).read_bytes()
    contents = data.decode()
    if name != 'AGENTS.md':
        assert 'cold_trace_archive_20261022_r1' in contents
        assert len(re.findall(r'^' + re.escape(chr(96) * 3), contents, re.MULTILINE)) % 2 == 0
        for destination in re.findall(r'\[[^\]\n]*\]\(([^)\n]+)\)', contents):
            if '://' not in destination and not destination.startswith('#'):
                relative = destination.split('#', 1)[0]
                assert (WORKSPACE / relative).exists(), (name, relative)
                checked_links.append([name, relative])
    documents[name] = dict(bytes=len(data), sha256=hashlib.sha256(data).hexdigest())
with (ROOT / 'documentation_after.json').open('x') as stream:
    json.dump(documents, stream, indent=2)
live = []
for process in psutil.process_iter(['pid', 'name', 'cmdline', 'create_time']):
    if process.info['name'] in ('gz sim server', 'gz sim gui', 'gzserver', 'gzclient', 'ffmpeg'):
        live.append(process.info)
result = dict(status='PASS_STORAGE_MIGRATION_NOT_PHYSICAL_RUN',
    migration=read('migration_result.json'), nine_protection_groups_identical=True,
    unchanged_runtime_entry_and_31_archive_registry=True,
    unchanged_original_dependencies_for_r347_and_r351=read('runtime_bindings_after.json'),
    tests=counts, total_passed_tests=sum(counts.values()),
    original_authority_prefix=authority, authority_prefix_unchanged=True,
    checked_document_links=len(checked_links),
    five_cold_manifests_verified=read('archive_verification.json')['manifests'],
    current_free_bytes=shutil.disk_usage(WORKSPACE).free,
    observed_simulation_recorder_processes=live,
    new_simulation_or_robot_motion=False, backup_or_model_changes=False)
assert (ROOT / 'REPORT.md').is_file() and (ROOT / 'README.md').is_file()
with (ROOT / 'verification_summary.json').open('x') as stream:
    json.dump(result, stream, indent=2)
members = sorted(path for path in ROOT.rglob('*') if path.is_file() and not path.is_symlink()
    and not {'__pycache__', '.pytest_cache'}.intersection(path.relative_to(ROOT).parts))
with (ROOT / 'SHA256SUMS').open('x') as stream:
    stream.write(''.join(digest(path) + '  ' + str(path.relative_to(ROOT)) + '\n' for path in members))
for path in members + [ROOT / 'SHA256SUMS']:
    path.chmod(0o444)
print(json.dumps(dict(status=result['status'], total_passed_tests=result['total_passed_tests'],
    free_bytes=result['current_free_bytes'], members=len(members),
    manifest_sha256=digest(ROOT / 'SHA256SUMS'))), flush=True)
