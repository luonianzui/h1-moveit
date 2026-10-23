import gzip
import hashlib
import json
from pathlib import Path

from cached_bucket import cached_bucket_corners
from qualify_bucket_cache import original_function, verify_source


ROOT = Path(__file__).resolve().parent


def main():
    verify_source()
    original = original_function()
    comparisons = 0
    kinds = {}
    with gzip.open(ROOT / 'run/loading_trace.jsonl.gz', 'rt') as stream:
        for line in stream:
            row = json.loads(line)
            payload = row.get('payload', {})
            poses = payload.get('poses') if isinstance(payload, dict) else None
            if not isinstance(poses, dict) or 'centrifuge' not in poses:
                continue
            for side in ('left', 'right'):
                if not all(name in poses for name in ('box48_' + side, 'bucket_' + side)):
                    continue
                expected = original(poses, side)
                actual = cached_bucket_corners(poses, side)
                assert actual.tobytes() == expected.tobytes(), (row['kind'], row.get('sim'), side)
                comparisons += 1
                kinds[row['kind']] = kinds.get(row['kind'], 0) + 1
    assert comparisons > 100
    with (ROOT / 'bucket_cache_actual_audit.json').open('x') as stream:
        json.dump(dict(
            status='PASS_ACTUAL_TRACE_CORNERS_EXACT_ORIGINAL_BYTES_NOT_WHOLE_TASK',
            comparisons=comparisons, event_kinds=kinds,
            original_function_sha256=hashlib.sha256(original.__code__.co_code).hexdigest(),
            runtime_cache_stats=json.loads((ROOT / 'run/bucket_cache_stats.json').read_text()),
            no_force_contact_or_guard_decimation=True,
            complete_task_separately_required=True,
        ), stream, indent=2)
    print('ACTUAL_CACHED_BUCKET_BYTES_PASS', comparisons, flush=True)


if __name__ == '__main__':
    main()
