import gzip
import hashlib
import json

import pytest

from trace_io import LosslessTrace, open_trace, verify_trace


def rows():
    return [json.dumps(dict(kind='contact', phase='test', sim=number, wall=1000 + number,
                           payload=dict(unicode='摇篮', depths=[.0001] * 20)), ensure_ascii=False) + '\n'
            for number in range(500)]


def test_lossless_all_original_jsonl_bytes_and_independent_hash_rows_crc(tmp_path):
    directory = tmp_path / 'run'
    directory.mkdir()
    writer = LosslessTrace(directory)
    source = ''.join(rows())
    for line in rows():
        writer.write(line)
        writer.flush()
    writer.close()
    writer.close()
    with open_trace(tmp_path) as stream:
        assert stream.read() == source
    result = verify_trace(tmp_path)
    assert result['status'] == 'PASS' and all(result['gates'].values())
    assert result['rows'] == 500 and result['decoded_sha256'] == hashlib.sha256(source.encode()).hexdigest()
    assert result['compressed_bytes'] < result['uncompressed_bytes'] / 4
    assert json.loads((directory / 'trace_progress.json').read_text())['rows'] == 500


def test_exclusive_output_refuses_reuse(tmp_path):
    (tmp_path / 'loading_trace.jsonl.gz').touch()
    with pytest.raises(FileExistsError):
        LosslessTrace(tmp_path)


@pytest.mark.parametrize('content', ['{}', '{}\n{}\n'])
def test_partial_or_multiple_row_write_is_rejected(tmp_path, content):
    writer = LosslessTrace(tmp_path)
    with pytest.raises(ValueError):
        writer.write(content)
    writer.close()


def test_legacy_read_only_and_ambiguous_trace_refusal(tmp_path):
    directory = tmp_path / 'run'
    directory.mkdir()
    path = directory / 'loading_trace.jsonl'
    path.write_text('{}\n')
    with open_trace(tmp_path) as stream:
        assert stream.read() == '{}\n'
    (directory / 'loading_trace.jsonl.gz').touch()
    with pytest.raises(ValueError):
        open_trace(tmp_path)


def test_mutated_metadata_cannot_pass(tmp_path):
    directory = tmp_path / 'run'
    directory.mkdir()
    writer = LosslessTrace(directory)
    writer.write(rows()[0])
    writer.close()
    path = directory / 'trace_metadata.json'
    metadata = json.loads(path.read_text())
    metadata['rows'] += 1
    path.write_text(json.dumps(metadata))
    assert verify_trace(tmp_path)['status'] == 'FAIL'


def test_truncated_gzip_footer_cannot_pass(tmp_path):
    directory = tmp_path / 'run'
    directory.mkdir()
    writer = LosslessTrace(directory)
    writer.write(rows()[0])
    writer.close()
    path = directory / 'loading_trace.jsonl.gz'
    path.write_bytes(path.read_bytes()[:-4])
    with pytest.raises(EOFError):
        verify_trace(tmp_path)
