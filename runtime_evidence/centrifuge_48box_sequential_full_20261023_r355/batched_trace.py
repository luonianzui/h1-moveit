import time

from trace_io import LosslessTrace


class BatchedLosslessTrace(LosslessTrace):
    block_bytes = 65536

    def __init__(self, directory):
        super().__init__(directory)
        self.pending = bytearray()

    def commit_pending(self):
        if self.pending:
            self.stream.write(self.pending)
            self.hasher.update(self.pending)
            self.pending.clear()

    def write(self, content):
        if self.closed:
            raise ValueError('Trace already closed')
        if not content.endswith('\n') or content.count('\n') != 1:
            raise ValueError('Exactly one complete JSONL row required')
        encoded = content.encode('utf-8')
        self.pending.extend(encoded)
        self.bytes += len(encoded)
        self.rows += 1
        self.latest = content
        if len(self.pending) >= self.block_bytes:
            self.commit_pending()
        return len(content)

    def flush(self, force=False):
        if self.closed or (not force and time.monotonic() - self.last_flush < 1):
            return
        self.commit_pending()
        return super().flush(force=force)
