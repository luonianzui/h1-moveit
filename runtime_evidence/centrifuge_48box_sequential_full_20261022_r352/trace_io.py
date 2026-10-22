import gzip
import hashlib
import json
from pathlib import Path
import time


class LosslessTrace:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.raw = (self.directory / 'loading_trace.jsonl.gz').open('xb')
        self.stream = gzip.GzipFile(fileobj=self.raw, mode='wb', compresslevel=1, mtime=0)
        self.hasher = hashlib.sha256()
        self.rows = 0
        self.bytes = 0
        self.latest = None
        self.last_flush = 0
        self.closed = False

    def write(self, content):
        if self.closed:
            raise ValueError('Trace already closed')
        if not content.endswith('\n') or content.count('\n') != 1:
            raise ValueError('Exactly one complete JSONL row required')
        encoded = content.encode('utf-8')
        self.stream.write(encoded)
        self.hasher.update(encoded)
        self.bytes += len(encoded)
        self.rows += 1
        self.latest = content
        return len(content)

    def flush(self, force=False):
        now = time.monotonic()
        if self.closed or (not force and now - self.last_flush < 1):
            return
        self.stream.flush()
        self.raw.flush()
        self.last_flush = now
        if self.latest is not None:
            row = json.loads(self.latest)
            progress = dict(kind=row['kind'], phase=row['phase'], sim=row['sim'], wall=row['wall'],
                            rows=self.rows, uncompressed_bytes=self.bytes,
                            compressed_bytes=self.raw.tell())
            temporary = self.directory / 'trace_progress.tmp'
            temporary.write_text(json.dumps(progress))
            temporary.replace(self.directory / 'trace_progress.json')

    def close(self):
        if self.closed:
            return
        self.flush(force=True)
        self.stream.close()
        self.raw.close()
        self.closed = True
        metadata = dict(status='CLOSED', compression='gzip_level_1', no_sampling=True,
                        utf8_sha256=self.hasher.hexdigest(), rows=self.rows,
                        uncompressed_bytes=self.bytes,
                        compressed_bytes=(self.directory / 'loading_trace.jsonl.gz').stat().st_size)
        with (self.directory / 'trace_metadata.json').open('x') as stream:
            json.dump(metadata, stream, indent=2)


def open_trace(root):
    root = Path(root) / 'run'
    compressed, original = root / 'loading_trace.jsonl.gz', root / 'loading_trace.jsonl'
    if compressed.exists() and original.exists():
        raise ValueError('Ambiguous trace paths')
    return gzip.open(compressed, 'rt', encoding='utf-8', newline='') if compressed.exists() else original.open(encoding='utf-8', newline='')


def verify_trace(root):
    root = Path(root)
    metadata = json.loads((root / 'run/trace_metadata.json').read_text())
    hasher = hashlib.sha256()
    rows = 0
    byte_count = 0
    with open_trace(root) as stream:
        for line in stream:
            assert line.endswith('\n')
            json.loads(line)
            encoded = line.encode('utf-8')
            hasher.update(encoded)
            rows += 1
            byte_count += len(encoded)
    gates = dict(closed=metadata['status'] == 'CLOSED', no_sampling=metadata['no_sampling'],
                 sha256=hasher.hexdigest() == metadata['utf8_sha256'],
                 rows=rows == metadata['rows'], bytes=byte_count == metadata['uncompressed_bytes'],
                 compressed_bytes=(root / 'run/loading_trace.jsonl.gz').stat().st_size == metadata['compressed_bytes'])
    return dict(status='PASS' if all(gates.values()) else 'FAIL', gates=gates,
                decoded_sha256=hasher.hexdigest(), rows=rows, uncompressed_bytes=byte_count,
                compressed_bytes=metadata['compressed_bytes'], gzip_crc_verified=True)
