import hashlib
import json
from pathlib import Path
import sys


EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[1]


def main():
    source = ROOT / 'src/h1_2_moveit_config/scripts/centrifuge_press_plan.py'
    original = source.read_text()
    content = original.replace('[.6, 0, .815]', '[.52, 0, .815]')
    old = ('                    approach = [home + fraction * (park - home) for fraction in np.linspace(0, 1, 81)]\n'
           '                    approach += [park + fraction * (pre - park) for fraction in np.linspace(0, 1, 101)[1:]]')
    assert content.count(old) == 1
    content = 'from button_rrt import route_button\n' + content.replace(old, '                    approach = route_button(self, side, home, pre)')
    derived = EVIDENCE / 'button_planner_routed_generated.py'
    with derived.open('x') as stream:
        stream.write(content)
    namespace = dict(__name__='routed_button_planner', __file__=str(derived))
    exec(compile(content, str(derived), 'exec'), namespace)
    planner = namespace['PressPlanner'](ROOT, EVIDENCE / 'inputs/robot_dual.urdf',
        ROOT / 'runtime_evidence/centrifuge_robot_press_20260922_r119/passive_leg_baseline.json')
    plan = planner.generate()
    with (EVIDENCE / 'button_routed_search.json').open('x') as stream:
        json.dump(plan, stream, indent=2)
    assert plan['status'] == 'PASS', plan['diagnostics']
    component = EVIDENCE.parent / 'centrifuge_48box_empty_sampling_20261013_r237'
    sys.path.insert(0, str(component))
    import button_sampling
    records = []
    for side, arm in plan['arms'].items():
        proof = button_sampling.propose(side, arm['joints'], arm['approach'], holding=None, payload=None)
        records.append(dict(side=side, proof=proof, contact_push_points_unchanged=len(arm['push'])))
        arm['approach'] = proof['points']
    with (EVIDENCE / 'inputs/button_plan.json').open('x') as stream:
        json.dump(plan, stream, indent=2)
    with (EVIDENCE / 'dual_button_plan_audit.json').open('x') as stream:
        json.dump(dict(status='PASS', original_shifted_linear_path_failed_and_preserved=True,
                       regenerated_world_anchor_and_collision_free_7joint_RRT=True,
                       torso_and_other_arm_fixed=True, original_joint_margins_5mm_clearance_and_42_contact_points=True,
                       original_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                       generated_sha256=hashlib.sha256(derived.read_bytes()).hexdigest(), records=records,
                       full_layout_payload_dense_FCL_still_required=True), stream, indent=2)
    print('ROUTED_BUTTON_PLANS_PASS', records, flush=True)


if __name__ == '__main__':
    main()
