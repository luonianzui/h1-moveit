import collections
import gzip
import json
from pathlib import Path

REVIEW = Path(__file__).resolve().parent
ROOT = REVIEW.parent / 'centrifuge_48box_sequential_full_20261022_r350'
recent = collections.deque(maxlen=160)
events = []
with gzip.open(ROOT / 'run/loading_trace.jsonl.gz', 'rt') as stream:
    for line in stream:
        row = json.loads(line)
        if row['kind'] == 'contact' and row['payload'].get('payload') == 'right':
            recent.append(row)
        if row['kind'] == 'sequential_remaining_basket_fault':
            events.append(dict(fault=row, recent_contacts=list(recent)))
        if row['kind'] in ('preload_observation_wait_started', 'preload_observation_wait_pass'):
            events.append(row)
assert any('fault' in item for item in events)
with (REVIEW / 'remaining_support_diagnostic.json').open('x') as stream:
    json.dump(events, stream, indent=2)
for item in events:
    if 'fault' in item:
        fault = item['fault']
        print(json.dumps(dict(phase=fault['phase'], sim=fault['sim'], metrics=fault['payload']['metrics'],
            continuity=fault['payload']['continuity'], contacts=fault['payload']['contacts']), indent=2))
