import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    logs = sorted(ROOT.glob('qualification*.log'))
    assert len(logs) == 6
    for path in logs:
        result = json.loads(path.read_text().splitlines()[-1])
        assert result['status'] == 'PASS_EXACT_CONTACT_RESPONSE_NOT_FULL_TASK'
        assert result['world_steps'] == result['contact_steps'] == 2400
        assert result['response_cases'] == 96
        assert result['candidate_calls'] > 0
    members = sorted(path for path in ROOT.rglob('*') if path.is_file() and not path.is_symlink()
        and not {'build', 'colcon_log', '__pycache__'}.intersection(path.relative_to(ROOT).parts))
    manifest = ROOT / 'SHA256SUMS'
    assert not manifest.exists()
    with manifest.open('x') as stream:
        stream.write(''.join(digest(path) + '  ' + str(path.relative_to(ROOT)) + '\n' for path in members))
    for path in members + [manifest]:
        path.chmod(0o444)
    print(json.dumps(dict(members=len(members), sha256=digest(manifest))), flush=True)


if __name__ == '__main__':
    main()
