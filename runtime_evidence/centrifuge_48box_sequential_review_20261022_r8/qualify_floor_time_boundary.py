import copy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parent
RUN = ROOT.parent / 'centrifuge_48box_sequential_full_20261022_r350'
source = (RUN / 'preload_floor_frame.py').read_text()
previous = '0 <= now-contact_stamp <= .001'
current = '0 <= round(now * 1e9) - round(contact_stamp * 1e9) <= 1_000_000'
assert source.count(previous) == 1
original, candidate = {}, {}
exec(compile(source, str(RUN / 'preload_floor_frame.py'), 'exec'), original)
exec(compile(source.replace(previous, current), '<offline-one-nanosecond-boundary-probe>', 'exec'), candidate)
record = json.loads((ROOT / 'r350_preload_fixture.json').read_text())
fault = record['fault']
history = original['FloorContactHistory']()
for row in record['contacts_before']:
    history.observe(row['payload'])
cycle = SimpleNamespace(current_grasp_hand='right', grasp_candidate='right', phase='right_retrieval_grasp',
    retrieval_closing=True, holders=SimpleNamespace(rows={}), pair_goal_handles={'right': SimpleNamespace(accepted=True)},
    preload_grasp_claim=record['claim']['payload'], accepted_gripper_targets={'right': record['goal']['payload']['target']},
    accepted_gripper_efforts={'right': record['goal']['payload']['feedforward_N']},
    sim=fault['sim'], joint_time=record['joints']['payload']['source_sim'],
    positions=record['joints']['payload']['positions'], poses=fault['payload']['poses'],
    contacts={'right': fault['payload']['contacts']}, closing_floor_history=history)
metrics = fault['payload']['metrics']
before = original['single_missing_floor_frame'](cycle, 'right', metrics)
after = candidate['single_missing_floor_frame'](cycle, 'right', metrics)
assert not before['valid'] and after['valid']
assert after['not_a_grip_certificate'] and after['missing_floor_sim_s'] <= .0005 + 1e-12
cases = []
stamp = history.rows['right']['stamp']
for offset_ns in (-1, 0, 999999, 1000000, 1000001):
    sample = copy.deepcopy(cycle)
    sample.sim = (round(stamp * 1e9) + offset_ns) / 1e9
    passed = candidate['single_missing_floor_frame'](sample, 'right', metrics)['valid']
    assert passed == (0 <= offset_ns <= 1_000_000)
    cases.append(dict(age_ns=offset_ns, valid=passed))
result = dict(status='PASS_EXACT_ONE_MILLISECOND_BOUNDARY_COMPONENT_ONLY_NOT_FULL_RUN',
    source_sha256=hashlib.sha256(source.encode()).hexdigest(),
    actual_fault_sim=cycle.sim, actual_contact_stamp=stamp, original_float_age=cycle.sim-stamp,
    exact_age_ns=round(cycle.sim*1e9)-round(stamp*1e9),
    original=before, candidate=after, boundary_cases=cases,
    no_new_support_contact_or_holder=True, original_one_frame_floor_history_and_midband_guards_unchanged=True,
    post_fault_next_observation_s=record['contacts_after'][0]['payload']['stamp'] - fault['sim'],
    post_fault_observation_not_uninterrupted_recovery=True)
with (ROOT / 'floor_time_boundary_qualification.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result, indent=2))
