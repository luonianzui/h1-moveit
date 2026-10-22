import json
from pathlib import Path

from trace_io import verify_trace


EVIDENCE = Path(__file__).resolve().parent
result = verify_trace(EVIDENCE)
with (EVIDENCE / 'trace_audit.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result), flush=True)
assert result['status'] == 'PASS'
