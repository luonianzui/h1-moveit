import copy
import itertools
import json
import math

import numpy as np
from scipy.spatial.transform import Rotation

from extraction_clearance import bucket_rim_bounds
from pilot_leveling import leveled_retrieval_pilot_target
from load_bound_reserve import InteriorLoadPlanner
from sequential_geometry import EVIDENCE, HANDS, SIZE, SinglePlanner, pose_matrix


def hints_for(kind, hand):
    return json.loads((EVIDENCE / 'inputs/sequential_hints.json').read_text())[kind][hand]


def nearest_symmetric_yaw(target, current):
    if not math.isfinite(target) or not math.isfinite(current):
        raise ValueError('Finite rigid basket/body yaw required')
    return current + (target - current + math.pi / 2) % math.pi - math.pi / 2


def home(request):
    planner = SinglePlanner(request)
    return planner, {name: request['positions'][name] for name in planner.names}


def result(planner, stages, reference=None):
    return dict(status='PASS_GEOMETRY_ONLY', stages=stages, checked_samples=planner.checked,
                maximum_joint_step_rad=.002, maximum_ik_position_error_m=planner.maximum_position_error,
                maximum_ik_angle_error_deg=planner.maximum_angle_error,
                single_hand=planner.hand, single_payload=planner.payload,
                planning_reference=None if reference is None else np.asarray(reference).tolist(),
                no_physical_attachment=True, runtime_reference_not_rebased=True,
                no_new_physical_execution_certificate=True)


def park(request):
    if request.get('original_runtime_holder_references'):
        raise ValueError('Parking is only allowed with empty hands')
    planner, first = home(request)
    target = json.loads((EVIDENCE / 'inputs/sequential_hints.json').read_text())['park']
    samples = planner.empty_connector(first, target)
    return result(planner, [dict(phase='empty_park', samples=samples)])


def neutral(request):
    if request.get('original_runtime_holder_references'):
        raise ValueError('Neutral return requires actually empty hands')
    planner, first = home(request)
    baseline = json.loads((EVIDENCE / 'inputs/plan.json').read_text())['initial_positions']
    target = {name: baseline[name] for name in first}
    samples = planner.empty_connector(first, target)
    return result(planner, [dict(phase='empty_neutral', samples=samples)])


def pickup(request, retrieval=False):
    if request.get('original_runtime_holder_references'):
        raise ValueError('Pickup planning must precede the actual grasp')
    hint = hints_for('retrieval' if retrieval else 'loading', request['hand'])
    reference = np.array(hint['reference'])
    planner = SinglePlanner(request, reference)
    first = {name: request['positions'][name] for name in planner.names}
    source = hint['initial'] if retrieval else hint['stages'][0]['samples'][0]
    proposal = dict(first, **{name: source[name] for name in planner.active})
    body = pose_matrix(request['poses']['box48_' + request['payload']])
    grip = planner.solve(body, proposal, refine=False)
    above = body.copy()
    above[2, 3] += .12
    if retrieval:
        source = hint['stages'][1]['samples'][-1]
    else:
        guides = hint['stages'][0]['samples']
        source = guides[round(.12 / (1.15975 - .84475) * (len(guides) - 1))]
    proposal = dict(first, **{name: source[name] for name in planner.active})
    pre = planner.solve(above, proposal, refine=False)
    approach = planner.empty_connector(first, pre)
    guides = [{name: pre[name] + fraction * (grip[name] - pre[name]) for name in pre}
              for fraction in np.linspace(0, 1, 81)]
    descend = planner.guided_stage(above, body, pre, 'empty_descend', guides)
    return result(planner, [dict(phase='empty_approach', samples=approach), descend], reference)


def load_transport(request, reference):
    hint = hints_for('loading', request['hand'])
    planner = InteriorLoadPlanner(request, reference)
    seed = {name: request['positions'][name] for name in planner.names}
    body = pose_matrix(request['poses']['box48_' + request['payload']])
    planner.attach(body, seed)
    high = body.copy()
    high[2, 3] = 1.15975
    bucket = pose_matrix(request['poses']['centrifuge']) @ pose_matrix(request['poses']['bucket_' + request['payload']])
    above = high.copy()
    above[:2, 3] = bucket[:2, 3]
    yaw = nearest_symmetric_yaw(math.atan2(bucket[1, 0], bucket[0, 0]) + math.pi / 2,
                                math.atan2(body[1, 0], body[0, 0]))
    above[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
    down = above.copy()
    down[2, 3] = 1.07525
    stages = []
    for index, (phase, target) in enumerate([('load_lift', high), ('load_transfer', above), ('load_lower', down)]):
        stage = planner.guided_stage(body, target, seed, phase, hint['stages'][index]['samples'],
                                     support_start='table' if index == 0 else None)
        stages.append(stage)
        body, seed = target, stage['samples'][-1]
    return result(planner, stages, reference)


def retrieve_transport(request, reference):
    hint = hints_for('retrieval', request['hand'])
    planner = SinglePlanner(request, reference)
    seed = {name: request['positions'][name] for name in planner.names}
    body = pose_matrix(request['poses']['box48_' + request['payload']])
    planner.attach(body, seed)
    bucket = pose_matrix(request['poses']['centrifuge']) @ pose_matrix(request['poses']['bucket_' + request['payload']])
    rim = bucket_rim_bounds(EVIDENCE.parents[1] / 'src/h1_2_description/models/centrifuge/model.sdf')[request['payload']]
    local = np.linalg.inv(bucket) @ body
    vertices = np.array(list(itertools.product((-1., 1.), repeat=3))) * np.array(SIZE) / 2
    bottom = (vertices @ local[:3, :3].T + local[:3, 3])[:, 2].min()
    axis_exit = body.copy()
    axis_exit[:3, 3] += (-bucket[:3, 0] * local[0, 3] - bucket[:3, 1] * local[1, 3]
                        + bucket[:3, 2] * max(0., float(rim + .006 - bottom)))
    high = body.copy()
    high[2, 3] = 1.15975
    outboard = high.copy()
    outboard[1, 3] = .32 if request['hand'] == 'left' else -.32
    yaw = math.pi if abs(math.atan2(body[1, 0], body[0, 0])) > math.pi / 2 else 0.
    table = outboard.copy()
    table[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
    table[:3, 3] = [.44, .38 if request['hand'] == 'left' else -.38, 1.13475 if request['hand'] == 'right' else 1.14475]
    down = table.copy()
    down[2, 3] = .84475
    stages = []
    for index, (phase, target) in enumerate([('retrieve_axis_exit', axis_exit), ('retrieve_lift', high),
                                            ('retrieve_outboard', outboard), ('retrieve_transfer', table), ('retrieve_setdown', down)]):
        guides = hint['stages'][index]['samples'][1:]
        stage = planner.guided_stage(body, target, seed, phase, guides, table_end=index == 4,
                                     support_start='basket' if index == 0 else None)
        stages.append(stage)
        body, seed = target, stage['samples'][-1]
    return dict(result(planner, stages, reference), table_yaw_rad=yaw)


def retreat(request):
    if request.get('original_runtime_holder_references'):
        raise ValueError('A qualified actual normal release must precede empty retreat')
    planner = SinglePlanner(request, np.eye(4))
    first = {name: request['positions'][name] for name in planner.names}
    wrist = planner.wrist(first)
    above = wrist.copy()
    above[2, 3] += .0845
    stage = planner.stage(wrist, above, first, 'empty_vertical_retreat', count=81, refine=False)
    future = copy.deepcopy(request)
    future['positions'].update(stage['samples'][-1])
    parked = park(future)
    return result(planner, [stage, *parked['stages']]) | dict(checked_samples=planner.checked + parked['checked_samples'])


def pilot(request, reference, retrieval=False):
    planner = SinglePlanner(request, reference)
    seed = {name: request['positions'][name] for name in planner.names}
    body = pose_matrix(request['poses']['box48_' + request['payload']])
    planner.attach(body, seed)
    target = body.copy()
    target[2, 3] += .015
    if retrieval:
        target = leveled_retrieval_pilot_target(body, request['poses'], request['payload'])
    stage = planner.stage(body, target, seed, 'single_pilot', count=21,
                          support_start='basket' if retrieval else 'table', refine=False)
    return result(planner, [stage], reference)
