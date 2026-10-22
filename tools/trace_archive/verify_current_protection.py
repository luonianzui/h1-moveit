#!/usr/bin/env python3
"""Run the unchanged r273 protection chain with outputs in this migration audit."""
import hashlib
import json
import os
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[2]
audit = root / 'runtime_evidence/trace_archive_20261014_r1'
parent = root / 'runtime_evidence/centrifuge_48box_initial_approach_20261014_r273'
source = parent / 'verify_dual_protection.py'
manifest = {line.split(maxsplit=1)[1].lstrip('*'):line.split()[0]
            for line in (parent / 'SHA256SUMS').read_text().splitlines()}
assert hashlib.sha256(source.read_bytes()).hexdigest() == manifest[source.name]
assert sys.argv[1] in ('before', 'after')
# This is the existing chain's supported new-evidence-root convention.
# It validates the original sources and emits new audit outputs without touching old runs.
exec(compile(source.read_text(), str(source), 'exec'),
     dict(__name__='__main__', __file__=str(audit / source.name)))
if sys.argv[1] == 'after':
    from compat import ENVIRONMENT, install
    from archive_cli import doctor
    result = doctor(install(os.environ[ENVIRONMENT]))
    assert result['status'] == 'PASS' and result['registered_files'] == 31
    with (audit / 'archive_doctor.json').open('x') as stream:
        json.dump(result, stream, indent=2)
    print('ARCHIVE_DOCTOR_PASS', result['registered_files'], flush=True)
