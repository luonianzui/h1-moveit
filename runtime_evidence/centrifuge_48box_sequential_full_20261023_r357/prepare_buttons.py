import hashlib
import json
from pathlib import Path
import sys


EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[1]


def main():
    source = ROOT / 'src/h1_2_moveit_config/scripts/centrifuge_press_plan.py'
    content = source.read_text()
    assert content.count('[.6, 0, .815]') == 2
    changed = content.replace('[.6, 0, .815]', '[.52, 0, .815]')
    assert changed.replace('[.52, 0, .815]', '[.6, 0, .815]') == content
    derived = EVIDENCE / 'button_planner_generated.py'
    with derived.open('x') as stream:
        stream.write(changed)
    namespace = dict(__name__='dual_button_planner', __file__=str(derived))
    exec(compile(changed, str(derived), 'exec'), namespace)
    planner = namespace['PressPlanner'](ROOT, EVIDENCE / 'inputs/robot_dual.urdf',
        ROOT / 'runtime_evidence/centrifuge_robot_press_20260922_r119/passive_leg_baseline.json')
    plan = planner.generate()
    assert plan['status'] == 'PASS', plan['diagnostics']
    with (EVIDENCE / 'inputs/button_plan_dense.json').open('x') as stream:
        json.dump(plan, stream, indent=2)
    component = EVIDENCE.parent / 'centrifuge_48box_empty_sampling_20261013_r237'
    sys.path.insert(0, str(component))
    import button_sampling
    provenance = []
    for side in ('left', 'right'):
        arm = plan['arms'][side]
        original = arm['approach']
        result = button_sampling.propose(side, arm['joints'], original, holding=None, payload=None)
        compressed = result['points']
        arm['approach'] = compressed
        provenance.append(dict(side=side, original_points=len(original), compressed_points=len(compressed), certificate=result,
                               push_points_unchanged=len(arm['push'])))
    with (EVIDENCE / 'inputs/button_plan.json').open('x') as stream:
        json.dump(plan, stream, indent=2)
    with (EVIDENCE / 'dual_button_plan_audit.json').open('x') as stream:
        json.dump(dict(status='PASS', original_formal_source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                       generated_source_sha256=hashlib.sha256(derived.read_bytes()).hexdigest(),
                       only_experimental_centrifuge_world_anchor_changed=True, current_robot_x_m=.12,
                       current_device_x_m=.52, old_r119_plan_not_replayed=True, compression=provenance,
                       fresh_physical_contact_and_dense_full_state_FCL_still_required=True), stream, indent=2)
    print('NEW_BUTTON_PLAN_AND_COMPRESSED_TRAVEL', provenance, flush=True)


if __name__ == '__main__':
    main()
