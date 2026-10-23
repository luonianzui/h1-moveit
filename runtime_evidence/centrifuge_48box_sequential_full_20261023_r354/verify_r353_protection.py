import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
EXPECTED = {
    'centrifuge_48box_sequential_full_20261022_r353': '541d8a5a1da0c245ca30ed2f84f065c1120b10e0ecf663168525d33a836d3ed2',
    'centrifuge_48box_sequential_review_20261022_r11': '1cb9c6bb83990e3099b99a18bb919236c8f16542688ce0db9ba478e98e583c21',
    'centrifuge_48box_sequential_review_20261022_r10': '049fe9cfbbf4399d33958cef482f466730d56ddb25f36367aef5f83aa356c127',
}
ERRATA = {
    'centrifuge_48box_sequential_review_20261022_r11': 'a67f2c17f5fd6c15bbd07221be6c6ab3123a745a4d877e6e08ddee98985d47ba',
    'centrifuge_48box_sequential_review_20261022_r10': '3d36bf1f88aae9f60cb4f03a61729322c49f60fa4a231258ca27c26e11905329',
}

def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

def main():
    phase = sys.argv[1]
    assert phase in ('before', 'after')
    records, exceptions = {}, []
    for name, expected_manifest in EXPECTED.items():
        folder = ROOT.parent / name
        manifest = folder / 'SHA256SUMS'
        assert digest(manifest) == expected_manifest
        lines = manifest.read_text().splitlines()
        for row in lines:
            expected, relative = row.split(maxsplit=1)
            member = folder / relative
            actual = digest(member)
            assert member.stat().st_mode & 0o777 == 0o444, member
            if actual != expected:
                assert relative == 'freeze_review.log' and name in ERRATA
                assert expected == hashlib.sha256(b'').hexdigest()
                assert actual == ERRATA[name]
                receipt = dict(members=len(lines), manifest_sha256=expected_manifest)
                assert member.read_text() == name + ' ' + str(receipt) + '\n'
                exceptions.append(dict(path=str(member), original_manifest_hash=expected, observed_hash=actual,
                    scope='sealer_stdout_written_after_its_own_hash_no_source_data_video_change'))
            records[str(member)] = actual
        records[str(manifest)] = expected_manifest
    assert len(exceptions) == 2
    result = dict(status='PASS_PRESERVED_WITH_PREEXISTING_REVIEW_LOG_ERRATUM',
                  source_execution_data_video_all_original_manifest_members_match=True,
                  sha256=records, preexisting_manifest_exceptions=exceptions,
                  old_files_not_rewritten=True)
    if phase == 'after':
        assert result == json.loads((ROOT / 'r353_protection_before.json').read_text())
    with (ROOT / f'r353_protection_{phase}.json').open('x') as stream:
        json.dump(result, stream, indent=2)
    print('R353_PRESERVED_WITH_TWO_PREEXISTING_REVIEW_STDOUT_LOG_EXCEPTIONS', phase, len(records))

if __name__ == '__main__':
    main()

