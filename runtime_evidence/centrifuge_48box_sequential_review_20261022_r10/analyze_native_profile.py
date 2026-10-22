import hashlib
import json
from pathlib import Path
import subprocess


REVIEW = Path(__file__).resolve().parent
WORKSPACE = REVIEW.parents[1]
RUN = REVIEW.parent / 'centrifuge_48box_sequential_full_20261022_r352/run'


def main():
    metadata = json.loads((RUN / 'main-thread.prof.complete').read_text())
    owner = json.loads((RUN / 'startup_review_freeze.json').read_text())['server']
    capture = json.loads((REVIEW / 'native_process_identity.json').read_text())
    assert metadata['status'] == 'PROFILE_STOPPED'
    assert metadata['pid'] == owner['pid'] == capture['pid']
    assert metadata['tid'] in capture['tids']
    assert metadata['samples'] >= 100 and metadata['preupdate_calls'] > 0
    assert metadata['no_simulation_commands']
    assert not (RUN / 'main-thread.prof.failure').exists()
    profiler = WORKSPACE / 'runtime_dependencies/gperftools_profile_20261015_r1/extracted/usr/bin/google-pprof'
    profile = RUN / 'main-thread.prof'
    commands = []
    for label, extra in (('flat', []), ('cumulative', ['--cum'])):
        arguments = [str(profiler), '--text', '--nodecount=60', *extra,
                     capture['executable'], str(profile)]
        with (REVIEW / ('native_profile_' + label + '.txt')).open('x') as output:
            completed = subprocess.run(arguments, stdout=output, stderr=subprocess.STDOUT, check=False)
        commands.append(dict(argv=arguments, returncode=completed.returncode))
        assert completed.returncode == 0
    with profile.open('rb') as stream:
        checksum = hashlib.file_digest(stream, 'sha256').hexdigest()
    result = dict(status='PASS_OWNED_PREUPDATE_CPU_SAMPLING_NOT_SPEED_CERTIFICATE',
        metadata=metadata, commands=commands, profile_sha256=checksum,
        sampled_thread_is_process_main=metadata['tid'] == metadata['pid'],
        other_threads_excluded=True, profiler_overhead_not_subtracted=True,
        sampled_counts_not_exact_elapsed_time=True, task_complete=json.loads((RUN / 'result.json').read_text())['status'] == 'PASS')
    with (REVIEW / 'native_profile_audit.json').open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
