import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


assert json.loads((ROOT / 'after_verification.json').read_text())['status'] == 'PASS_BOUNDED_INTERMEDIATE_CACHE_CLEANUP'
assert (ROOT / 'REPORT.md').is_file()
members = sorted(path for path in ROOT.iterdir() if path.is_file() and not path.is_symlink())
with (ROOT / 'SHA256SUMS').open('x') as stream:
    stream.write(''.join(digest(path) + '  ' + path.name + '\n' for path in members))
for path in members + [ROOT / 'SHA256SUMS']:
    path.chmod(0o444)
print(json.dumps(dict(status='SEALED', members=len(members), manifest_sha256=digest(ROOT / 'SHA256SUMS'))), flush=True)
