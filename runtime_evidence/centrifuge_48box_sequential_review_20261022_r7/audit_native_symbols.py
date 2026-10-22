import bisect
import collections
import hashlib
import json
from pathlib import Path
import struct
import subprocess


REVIEW = Path(__file__).resolve().parent
RUN = REVIEW.parent / 'centrifuge_48box_sequential_full_20261022_r349/run'


def main():
    mappings = []
    bases = {}
    for line in (REVIEW / 'native_process_maps.txt').read_text().splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) != 6 or not fields[5].startswith('/'):
            continue
        lower, upper = [int(value, 16) for value in fields[0].split('-')]
        path = fields[5]
        bases[path] = min(bases.get(path, lower), lower - int(fields[2], 16))
        if 'x' in fields[1]:
            mappings.append((lower, upper, path))
    mappings.sort()
    starts = [row[0] for row in mappings]
    names = ('Physics::Update', 'PhysicsPrivate::Step', 'WorldForwardStep',
             'World::step()', 'World::bake()', 'BoxedLcpConstraintSolver::solveConstrainedGroup',
             'ConstraintSolver::solveConstrainedGroups', 'LocalResourceRetriever::exists')
    symbols = []
    hashes = {}
    for path in bases:
        if not any(token in path for token in ('libdart.so.', 'libDartConstraintCache.so', 'libDartContactResponse.so', 'libgz-sim8-physics-system', 'libgz-physics7-dartsim')):
            continue
        output = subprocess.check_output(['nm', '-D', '--print-size', '--defined-only', '-C', path], text=True)
        with (REVIEW / (Path(path).name + '.dynamic_symbols.txt')).open('x') as stream:
            stream.write(output)
        with Path(path).open('rb') as stream:
            hashes[path] = hashlib.file_digest(stream, 'sha256').hexdigest()
        for line in output.splitlines():
            fields = line.split(maxsplit=3)
            if len(fields) == 4 and any(name in fields[3] for name in names):
                address, size = int(fields[0], 16), int(fields[1], 16)
                if size:
                    symbols.append(dict(name=fields[3], library=path, lower=bases[path] + address,
                                        upper=bases[path] + address + size, bytes=size,
                                        flat=0, cumulative=0))
    data = (RUN / 'main-thread.prof').read_bytes()
    header = struct.unpack_from('<5Q', data)
    assert header[:3] == (0, 3, 0)
    offset = 40
    total = 0
    flat_libraries, cumulative_libraries = collections.Counter(), collections.Counter()
    while offset + 16 <= len(data):
        count, depth = struct.unpack_from('<2Q', data, offset)
        offset += 16
        assert 0 < depth < 10000
        stack = struct.unpack_from('<' + str(depth) + 'Q', data, offset)
        offset += depth * 8
        if count == 0 and stack == (0,):
            break
        total += count
        addresses = [address - (index > 0) for index, address in enumerate(stack)]
        libraries = []
        for address in addresses:
            position = bisect.bisect_right(starts, address) - 1
            mapping = mappings[position] if position >= 0 else None
            libraries.append(mapping[2] if mapping and address < mapping[1] else 'unmapped')
        flat_libraries[libraries[0]] += count
        cumulative_libraries.update({name: count for name in set(libraries)})
        for symbol in symbols:
            if symbol['lower'] <= addresses[0] < symbol['upper']:
                symbol['flat'] += count
            if any(symbol['lower'] <= address < symbol['upper'] for address in addresses):
                symbol['cumulative'] += count
    assert total == json.loads((RUN / 'main-thread.prof.complete').read_text())['samples']
    for symbol in symbols:
        symbol['flat_pct'] = 100 * symbol['flat'] / total
        symbol['cumulative_pct'] = 100 * symbol['cumulative'] / total
    result = dict(status='PASS_EXACT_EXPORTED_SYMBOL_SIZE_AND_PROCESS_MAP_AUDIT', samples=total,
        sample_period_us=header[3], binary_sha256=hashes,
        symbols=sorted(symbols, key=lambda row: row['cumulative'], reverse=True),
        flat_libraries=flat_libraries.most_common(15), cumulative_libraries=cumulative_libraries.most_common(15),
        caveat='Only exact exported symbol address ranges counted. Unexported local functions remain unresolved; nearest-symbol pprof labels may be wrong. Inclusive percentages overlap and are not wall time.')
    with (REVIEW / 'native_symbol_audit.json').open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
