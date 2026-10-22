import json


def latest_world_stats(contents):
    if not isinstance(contents, str) or len(contents) > 262144:
        raise ValueError('Bounded text statistics required')
    decoder = json.JSONDecoder()
    remaining = contents.strip()
    previous = None
    records = []
    while remaining:
        row, end = decoder.raw_decode(remaining)
        if not isinstance(row, dict) or not isinstance(row.get('simTime'), dict):
            raise ValueError('Every statistics record must contain a simulation timestamp')
        stamp = row['simTime']
        if not any(name in stamp for name in ('sec', 'nsec')):
            raise ValueError('Missing actual simulation timestamp')
        components = [stamp.get(name, 0) for name in ('sec', 'nsec')]
        if any(isinstance(value, bool) or not isinstance(value, (int, str)) for value in components):
            raise ValueError('Integral protobuf time fields required')
        seconds, nanoseconds = map(int, components)
        if seconds < 0 or not 0 <= nanoseconds < 1000000000:
            raise ValueError('Invalid simulation timestamp')
        current = seconds * 1000000000 + nanoseconds
        if previous is not None and current < previous:
            raise ValueError('Simulation statistics time regressed')
        records.append(row)
        if len(records) > 64:
            raise ValueError('Unexpected statistics stream length')
        previous = current
        remaining = remaining[end:].lstrip()
    if not records:
        raise ValueError('Empty simulation statistics')
    return records[-1]
