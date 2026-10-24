import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[1]
MANIFESTS = {
    'centrifuge_48box_elbow_full_20261016_r291/SHA256SUMS': '3988cd0eac7a1181c465e07a726fa3d07731fa5556657704209db22b13e1e153',
    'centrifuge_48box_redline_probe_20261017_r1/SHA256SUMS': '0cc796df23cc806b04fd4a6e1a67e8d241097c689fd96628ea52777184a73b51',
    'centrifuge_48box_elbow_independent_review_20261016_r1/SHA256SUMS': '3ddf81399ddfc2c544db394a80d3cce8c9d85527f648b8e895261ff3e88f1986',
    'centrifuge_48box_elbow_briefing_20261016_r1/SHA256SUMS_revision2': '642b95549021ad4f939187b2964ab255b6b9db0528ac58cc477da9a01fb9d19e',
}
MODELS = {
    'incubator': '44ff23fc7077f0c232e2e69ff9d73aba336140d6a7951968c9c4241a97e73db9',
    'centrifuge': 'f017dfcf10b0e2c583e80bb6f625ad5998c7815f8fa69e3c1eaa7134a3684a55',
    'lab_48box': 'dc582ab7ef80536e28836c743f9671b2709d54143cb1d3c2a87a0e3c5f9bf1ec',
}


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    records, failures = [], []
    counts = {}
    for relative, expected_manifest in MANIFESTS.items():
        manifest = ROOT.parent / relative
        if digest(manifest) != expected_manifest:
            raise ValueError('Unexpected original sealed manifest: ' + relative)
        rows = manifest.read_text().splitlines()
        counts[relative] = len(rows)
        for row in rows:
            expected, filename = row.split(maxsplit=1)
            path = manifest.parent / filename.lstrip('*')
            actual = digest(path)
            record = dict(path=str(path), expected=expected, actual=actual)
            records.append(record)
            if actual != expected:
                failures.append(record)
    models = {}
    for name, expected in MODELS.items():
        path = WORKSPACE / f'src/h1_2_description/models/{name}/model.sdf'
        models[name] = digest(path)
        if models[name] != expected:
            failures.append(dict(path=str(path), expected=expected, actual=models[name]))
    inventory = Path('/home/liu/h1_moveit/backups/backup_1009_release_20261009/archive_payload_sha256.json')
    source_count = 0
    for relative, expected in json.loads(inventory.read_text()).items():
        if relative.startswith('src/'):
            source_count += 1
            path = WORKSPACE / relative
            actual = digest(path) if path.is_file() else 'MISSING'
            if actual != expected:
                failures.append(dict(path=str(path), expected=expected, actual=actual))
    assert source_count == 785
    result = dict(status='PASS' if not failures else 'FAIL', counts=counts, models=models,
                  official_source_members=source_count, records=records, failures=failures,
                  scope='OFFLINE_CANDIDATE_PROTECTION_NOT_SEVEN_GROUP_FRESH_RUNTIME_GATE',
                  backup_archives_not_written_or_rehashed=True)
    with (ROOT / 'protected_offline_after.json').open('x') as stream:
        json.dump(result, stream, indent=2)
    print('OFFLINE_PROTECTION', result['status'], counts, source_count, models, flush=True)
    assert not failures, failures


if __name__ == '__main__':
    main()
