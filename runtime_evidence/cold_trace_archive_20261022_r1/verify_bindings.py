import hashlib
import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
records = []
for name in ('centrifuge_48box_sequential_full_20261022_r347',
             'centrifuge_48box_sequential_full_20261022_r351'):
    folder = ROOT.parent / name
    source = folder / 'archive_runtime_binding.py'
    identity = json.loads((folder / 'identity.json').read_text())
    assert hashlib.sha256(source.read_bytes()).hexdigest() == identity['sha256'][source.name]
    specification = importlib.util.spec_from_file_location('binding_' + name, source)
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    count = module.verify_bound_dependencies()
    assert count == 36
    records.append(dict(run=name, unchanged_bound_dependencies=count))
with (ROOT / 'runtime_bindings_after.json').open('x') as stream:
    json.dump(dict(status='PASS', records=records, ordinary_runtime_wrapper=True), stream, indent=2)
print('ORIGINAL_RUNTIME_36_DEPENDENCIES_UNCHANGED', flush=True)
