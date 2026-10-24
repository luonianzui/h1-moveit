import ast
import hashlib
import json
import os
from pathlib import Path


REVIEW = Path(__file__).resolve().parent
source = ast.parse((REVIEW / 'freeze_results.py').read_text())
definitions = [item for item in source.body if isinstance(item, ast.FunctionDef) and item.name in ('digest', 'seal')]
namespace = dict(hashlib=hashlib, os=os, Path=Path)
exec(compile(ast.Module(body=definitions, type_ignores=[]), str(REVIEW / 'freeze_results.py'), 'exec'), namespace)
geometry = REVIEW.parent / 'centrifuge_48box_exit_clearance_20261024_r1'
cleanup = REVIEW.parent / 'pip_download_cache_cleanup_20261024_r1'
assert '9 passed' in (geometry / 'tests.log').read_text()
assert len(json.loads((geometry / 'centered_exit_qualification.json').read_text())['records']) == 3
assert json.loads((cleanup / 'result.json').read_text())['removed_bytes'] == 3021341614
records = {folder.name: namespace['seal'](folder) for folder in (geometry, cleanup)}
with (REVIEW / 'auxiliary_seals.json').open('x') as stream:
    json.dump(records, stream, indent=2)
