import ast
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent

def seal_function():
    tree = ast.parse((ROOT / 'freeze_results.py').read_text())
    definitions = [item for item in tree.body if isinstance(item, ast.FunctionDef) and item.name in ('seal', 'digest')]
    return ast.unparse(ast.Module(body=definitions, type_ignores=[]))

def test_refuses_self_hashed_stdout(tmp_path):
    target = tmp_path / 'candidate'
    target.mkdir()
    source = 'import os,hashlib\nfrom pathlib import Path\n' + seal_function() + '\nseal(Path(' + repr(str(target)) + '))'
    with (target / 'stdout.log').open('x') as stream:
        result = subprocess.run([sys.executable, '-c', source], stdout=stream, stderr=subprocess.PIPE)
    assert result.returncode != 0
    assert b'Seal output must not modify its own sealed tree' in result.stderr
    assert not (target / 'SHA256SUMS').exists()

def test_pipe_stdout_and_exact_members(tmp_path):
    target = tmp_path / 'candidate'
    target.mkdir()
    (target / 'evidence.txt').write_text('unchanged')
    source = 'import os,hashlib\nfrom pathlib import Path\n' + seal_function() + '\nseal(Path(' + repr(str(target)) + '))'
    result = subprocess.run([sys.executable, '-c', source], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    expected = hashlib.sha256(b'unchanged').hexdigest()
    assert (target / 'SHA256SUMS').read_text() == expected + '  evidence.txt\n'
    assert (target / 'evidence.txt').stat().st_mode & 0o777 == 0o444

