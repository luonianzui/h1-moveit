import gzip
import hashlib
import json

import pytest

from batched_trace import BatchedLosslessTrace
from trace_io import verify_trace


def test_actual_runtime_uses_batch_writer_without_changing_original_reader():
    import execute_loading
    from trace_io import LosslessTrace
    assert execute_loading.LosslessTrace is BatchedLosslessTrace
    assert BatchedLosslessTrace.close is LosslessTrace.close


@pytest.mark.parametrize('rows,large', [(0, False), (1, False), (700, False), (3, True)])
def test_original_bytes_hash_and_progress_preserved(tmp_path, rows, large):
    directory = tmp_path / 'run'
    directory.mkdir()
    writer = BatchedLosslessTrace(directory)
    source = []
    for number in range(rows):
        content = json.dumps(dict(kind='actual', phase='test', sim=number, wall=number,
                                  payload='真实反馈' * (20000 if large else 30)), ensure_ascii=False) + '\n'
        source.append(content)
        assert writer.write(content) == len(content)
        assert len(writer.pending) < writer.block_bytes
        writer.flush()
    writer.close()
    writer.close()
    encoded = ''.join(source).encode()
    assert gzip.decompress((directory / 'loading_trace.jsonl.gz').read_bytes()) == encoded
    audit = verify_trace(tmp_path)
    assert audit['status'] == 'PASS'
    assert audit['decoded_sha256'] == hashlib.sha256(encoded).hexdigest()
    assert audit['rows'] == rows
    if rows:
        assert json.loads((directory / 'trace_progress.json').read_text())['rows'] == rows
    with pytest.raises(ValueError):
        writer.write('{}\n')


def test_force_flush_commits_partial_block(tmp_path):
    writer = BatchedLosslessTrace(tmp_path)
    line = '{"kind":"actual","phase":"test","sim":1,"wall":1}\n'
    writer.write(line)
    assert writer.pending
    writer.flush(force=True)
    assert not writer.pending
    assert writer.hasher.hexdigest() == hashlib.sha256(line.encode()).hexdigest()
    writer.close()
    with pytest.raises(FileExistsError):
        BatchedLosslessTrace(tmp_path)


@pytest.mark.parametrize('content', ['{}', '{}\n{}\n'])
def test_no_partial_or_multiple_rows(tmp_path, content):
    writer = BatchedLosslessTrace(tmp_path)
    with pytest.raises(ValueError):
        writer.write(content)
    assert writer.rows == 0 and not writer.pending
    writer.close()


def test_only_retrieval_pilot_is_slowed():
    from speed_policy import motion_limits
    assert motion_limits('seq_retrieval_single_pilot', True) == (.1, .04)
    assert motion_limits('seq_load_pilot', True) == (.15, .04)
    assert motion_limits('seq_retrieval_retrieve_axis_exit', True) == (.15, .04)
    assert motion_limits('seq_retrieval_retrieve_transfer', True) == (.8, .08)
