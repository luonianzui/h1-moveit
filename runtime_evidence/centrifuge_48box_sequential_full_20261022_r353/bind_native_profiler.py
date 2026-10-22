import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[1]
SOURCE = ROOT.parent / 'centrifuge_48box_native_profile_20261021_r1'
LIBRARY = WORKSPACE / 'install_lab_scene_48box_native_profile_20261021_r1/h12_task_cpu_profile_r1/lib/libTaskCpuProfile.so'
assert (SOURCE / 'conda_six_before.txt').read_bytes() == (SOURCE / 'conda_six_after.txt').read_bytes()
records = {}
for source, name in [(LIBRARY, 'libTaskCpuProfile.so'),
                     (SOURCE / 'plugin_src/TaskCpuProfile.cc', 'TaskCpuProfile.cc')]:
    data = source.read_bytes()
    with (ROOT / 'inputs' / name).open('xb') as stream:
        stream.write(data)
    records[str(source.resolve())] = hashlib.sha256(data).hexdigest()
profiler = WORKSPACE / 'runtime_dependencies/gperftools_profile_20261015_r1/extracted/usr/lib/x86_64-linux-gnu/libprofiler.so'
records[str(profiler.resolve())] = hashlib.sha256(profiler.read_bytes()).hexdigest()
with (ROOT / 'native_profiler_dependencies.json').open('x') as stream:
    json.dump(dict(status='BUILT_OBSERVER_NOT_YET_RUNTIME_QUALIFIED', sha256=records,
                   no_global_injection_or_physics_changes=True), stream, indent=2)
