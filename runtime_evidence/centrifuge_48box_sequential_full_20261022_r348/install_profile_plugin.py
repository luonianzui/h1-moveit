import hashlib
import json
import subprocess
import time
from pathlib import Path

from startup_review_hold import require_scene


ROOT = Path(__file__).resolve().parent


def request_command(root):
    plugin = dict(name='TaskCpuProfile', filename=str(root / 'inputs/libTaskCpuProfile.so'))
    inner = ''.join('<' + key + '>' + str(root / 'run' / value) + '</' + key + '>'
                    for key, value in [('profile_path', 'main-thread.prof'),
                                       ('ready_path', 'process_identities.json'), ('done_path', 'result.json')])
    request = 'entity { name: "h1_world" type: WORLD } plugins { name: "TaskCpuProfile" filename: ' + json.dumps(plugin['filename']) + ' innerxml: ' + json.dumps(inner) + ' }'
    return ['gz', 'service', '-s', '/world/h1_world/entity/system/add',
            '--reqtype', 'gz.msgs.EntityPlugin_V', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000', '--req', request]


def main():
    require_scene(ROOT)
    identity = json.loads((ROOT / 'identity.json').read_text())
    for relative, expected in identity['sha256'].items():
        assert hashlib.sha256((ROOT / relative).read_bytes()).hexdigest() == expected
    dependencies = json.loads((ROOT / 'native_profiler_dependencies.json').read_text())['sha256']
    for name, expected in dependencies.items():
        assert hashlib.sha256(Path(name).read_bytes()).hexdigest() == expected
    assert not (ROOT / 'run/process_identities.json').exists()
    assert not (ROOT / 'run/execution.claim').exists()
    assert not (ROOT / 'run/main-thread.prof.claim').exists()
    with (ROOT / 'run/native_profiler_install.claim').open('x') as stream:
        json.dump(dict(status='ONCE_NEW_SCENE_OBSERVER_ONLY', command=request_command(ROOT)), stream)
    command = request_command(ROOT)
    response = subprocess.run(command, text=True, capture_output=True, timeout=10)
    with (ROOT / 'run/native_profiler_install.json').open('x') as stream:
        json.dump(dict(command=command, returncode=response.returncode,
                       stdout=response.stdout, stderr=response.stderr,
                       no_robot_model_or_control_commands=True), stream, indent=2)
    assert response.returncode == 0 and response.stdout.strip() == 'data: true'
    deadline = time.monotonic() + 5
    while not (ROOT / 'run/main-thread.prof.claim').exists() and time.monotonic() < deadline:
        time.sleep(.05)
    assert (ROOT / 'run/main-thread.prof.claim').exists()
    assert not (ROOT / 'run/main-thread.prof').exists()
    print('READ_ONLY_NATIVE_OBSERVER_CONFIGURED_BEFORE_EXECUTOR', flush=True)


if __name__ == '__main__':
    main()
