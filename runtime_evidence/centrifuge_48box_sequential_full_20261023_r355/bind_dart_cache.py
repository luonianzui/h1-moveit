import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT.parent / 'centrifuge_48box_dart_inertia_20261023_r1'
WORKSPACE = ROOT.parents[1]
library = WORKSPACE / 'install_lab_scene_48box_dart_inertia_20261023_r1/h12_dart_inertia_r1/lib/libDartConstraintCache.so'
assert (SOURCE / 'conda_six_before.txt').read_bytes() == (SOURCE / 'conda_six_after.txt').read_bytes()
rows = (SOURCE / 'solver_revision2.log').read_text().splitlines()
result = json.loads(next(row for row in reversed(rows) if row.startswith('{"status"')))
assert result['status'] == 'PASS_EXACT_SOLVER_COMPONENT_NOT_FULL_TASK'
assert result['matrix_cases'] == 105 and result['real_world_steps'] == 1200
assert result['real_contact_steps'] > 100 and result['component_speedup'] > 1.
query_rows = (SOURCE / 'queries_revision2.log').read_text().splitlines()
query_result = json.loads(next(row for row in reversed(query_rows) if row.startswith('{"status"')))
assert query_result['status'] == 'PASS_EXACT_GROUP_QUERIES_NOT_FULL_TASK'
assert query_result['world_steps'] == 5400 and query_result['vector_checks'] == 43200
assert query_result['retained_after_solve'] == 0 and query_result['component_speedup'] > 1.
dirty_result = json.loads((SOURCE / 'dirty_revision2.log').read_text().splitlines()[-1])
assert dirty_result['status'] == 'PASS_DIRTY_INVALIDATION_INSIDE_SOLVE'
assert dirty_result['checks'] == 1266 and dirty_result['dirty_refreshes'] >= 60
assert dirty_result['mode_rejections'] == 30 and dirty_result['retained'] == 0
records = {}
inertia_rows = (SOURCE / 'inertia_revision2.log').read_text().splitlines()
inertia_result = json.loads(next(row for row in reversed(inertia_rows) if row.startswith('{"status"')))
assert inertia_result['status'] == 'PASS_EXACT_INERTIA_QUERIES_NOT_FULL_TASK'
assert inertia_result['world_steps'] == 5400 and inertia_result['vector_checks'] == 43200
assert inertia_result['retained_after_solve'] == 0 and inertia_result['inertia_queries'] > 10000
assert inertia_result['component_speedup'] > 1.
for source, name in [(library, 'libDartConstraintCache.so'),
                     (SOURCE / 'plugin_src/CachedSolver.cc', 'CachedSolver.cc')]:
    data = source.read_bytes()
    destination = ROOT / 'inputs' / name
    if destination.exists():
        if destination.read_bytes() != data:
            parent = ROOT.parent / 'centrifuge_48box_sequential_full_20261023_r354/inputs' / name
            assert destination.read_bytes() == parent.read_bytes() and not (ROOT / 'identity.json').exists()
            destination.write_bytes(data)
    else:
        with destination.open('xb') as stream:
            stream.write(data)
    records[str(source.resolve())] = hashlib.sha256(data).hexdigest()
for source in [SOURCE / 'plugin_src/QualifyDirty.cc', SOURCE / 'dirty_revision2.log',
               SOURCE / 'plugin_src/QualifyInertia.cc', SOURCE / 'inertia_revision2.log', SOURCE / 'inertia_repeat_revision2.log',
               SOURCE / 'plugin_src/QualifySolver.cc', SOURCE / 'solver_revision2.log', SOURCE / 'plugin_src/QualifyQueries.cc',
               SOURCE / 'queries_revision2.log', SOURCE / 'UpstreamBodyNode.cpp', SOURCE / 'UpstreamContactConstraint.cpp',
               SOURCE / 'UpstreamBoxedLcpConstraintSolver.cpp',
               Path('/opt/ros/jazzy/opt/gz_dartsim_vendor/lib/libdart.so.6.13.2'),
               Path('/opt/ros/jazzy/share/ros_gz_sim/launch/gz_sim.launch.py')]:
    records[str(source)] = hashlib.sha256(source.read_bytes()).hexdigest()
for source in sorted(Path('/opt/ros/jazzy/opt/gz_dartsim_vendor/include/dart/constraint').glob('*.hpp')):
    records[str(source)] = hashlib.sha256(source.read_bytes()).hexdigest()
for source in [ROOT / 'inputs/libDartConstraintCache.so', ROOT / 'inputs/gz_solver.launch.py']:
    records[str(source)] = hashlib.sha256(source.read_bytes()).hexdigest()
with (ROOT / 'dart_cache_dependencies.json').open('x') as stream:
    json.dump(dict(status='PASS_OFFLINE_EXACT_MATRIX_AND_CONTACT_WORLD_NOT_FULL_TASK', sha256=records,
                   qualification=result, queries_qualification=query_result, inertia_qualification=inertia_result, dirty_qualification=dirty_result,
                   injection_scope='owned_new_gazebo_tree_only_no_global_environment',
                   physical_timestep_contacts_friction_mass_solver_bounds_unchanged=True,
                   optimization='r354_plus_same_dirty_flag_inertia_accessor_with_solve_scoped_owner_no_numeric_inertia_cache'),
              stream, indent=2)
print('DART_CACHE_BOUND_NEW_IDENTITY_ONLY', len(records), flush=True)
