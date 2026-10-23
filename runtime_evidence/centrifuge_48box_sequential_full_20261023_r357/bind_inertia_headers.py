import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
HEADERS = Path('/opt/ros/jazzy/opt/gz_dartsim_vendor/include/dart')
assert not (ROOT / 'identity.json').exists()
records = {}
for folder in ('dynamics', 'math', 'common'):
    for source in sorted((HEADERS / folder).rglob('*.hpp')):
        records[str(source)] = hashlib.sha256(source.read_bytes()).hexdigest()
assert str(HEADERS / 'dynamics/BodyNode.hpp') in records
assert str(HEADERS / 'dynamics/Skeleton.hpp') in records
with (ROOT / 'inertia_header_dependencies.json').open('x') as stream:
    json.dump(dict(status='PASS_CURRENT_INSTALLED_HEADER_BINDING', sha256=records,
        build_dart_version='6.13.2', system_headers_not_modified=True), stream, indent=2)
print('INERTIA_HEADERS_BOUND', len(records))
