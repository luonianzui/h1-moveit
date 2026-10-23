import copy
import itertools
import math
import time

import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation


ELBOW_CONTRACT = 'paired_elbow_up_fixed_grasp_v1'
ACTIVE_MODES = frozenset(('load_pickup', 'retrieval_pickup', 'pilot', 'load_transport',
                          'load_alignment', 'retrieval_transport'))


def active(request):
    marker = request.get('elbow_posture_contract')
    if marker is not None and marker != ELBOW_CONTRACT:
        raise ValueError('Unknown elbow posture contract')
    return marker == ELBOW_CONTRACT and request.get('mode') in ACTIVE_MODES


def wrist_errors(planner, targets, vector):
    if not np.isfinite(vector).all():
        raise ValueError('Nonfinite elbow IK joints')
    planner.planner.state.joint_positions = dict(planner.initial, **dict(zip(planner.names, vector.tolist())))
    planner.planner.state.update()
    errors = []
    for hand in ('left', 'right'):
        wrist = planner.planner.state.get_global_link_transform(hand + '_wrist_yaw_link')
        errors.extend(20 * (wrist[:3, 3] - targets[hand][:3, 3]))
        errors.extend(Rotation.from_matrix(targets[hand][:3, :3] @ wrist[:3, :3].T).as_rotvec())
    return np.array(errors)


def accurate(errors):
    return bool(np.asarray(errors).shape == (12,) and np.isfinite(errors).all() and
                all(np.linalg.norm(errors[offset:offset + 3]) / 20 <= .00002 and
                    math.degrees(float(np.linalg.norm(errors[offset + 3:offset + 6]))) <= .01
                    for offset in (0, 6)))


def posture_metrics(planner, joints):
    planner.planner.state.joint_positions = dict(planner.initial, **joints)
    planner.planner.state.update()
    hands = {}
    for hand in ('left', 'right'):
        elbow = planner.planner.state.get_global_link_transform(hand + '_elbow_link')[:3, 3]
        wrist = planner.planner.state.get_global_link_transform(hand + '_wrist_yaw_link')[:3, 3]
        vector = elbow - wrist
        length = float(np.linalg.norm(vector))
        if length < 1e-9 or not math.isfinite(length):
            raise ValueError('Degenerate elbow-to-wrist direction')
        hands[hand] = dict(elbow_xyz_m=elbow.tolist(), wrist_xyz_m=wrist.tolist(),
                           from_down_deg=math.degrees(math.acos(float(np.clip(vector[2] / length, -1, 1)))))
    return hands


def refine(planner, bodies, seed, original_solver):
    if not active(planner.request):
        return original_solver(bodies, seed)
    solved = original_solver(bodies, seed)
    targets = {hand: np.asarray(body) @ np.linalg.inv(planner.references[hand]) for hand, body in bodies.items()}
    origin = np.array([solved[name] for name in planner.names])
    lower = np.maximum(planner.lower, origin - .045)
    upper = np.minimum(planner.upper, origin + .045)

    def constraint(vector):
        return wrist_errors(planner, targets, vector)

    def objective(vector):
        constraint(vector)
        score = .00004 * float(np.sum((vector - origin) ** 2))
        for hand in ('left', 'right'):
            elbow = planner.planner.state.get_global_link_transform(hand + '_elbow_link')[:3, 3]
            preferred = targets[hand][:3, 3] + [0, 0, .24]
            score += float(np.sum((elbow - preferred) ** 2))
        return score

    candidate = minimize(objective, origin, method='SLSQP', bounds=list(zip(lower, upper)),
                         constraints=[dict(type='eq', fun=constraint)],
                         options=dict(maxiter=45, ftol=1e-10))
    if candidate.success and accurate(constraint(candidate.x)) and objective(candidate.x) < objective(origin):
        return dict(zip(planner.names, candidate.x.tolist()))
    return solved


def pickup_seeds(worker, planner, request):
    guide = worker.json.loads((worker.EVIDENCE / 'inputs/dual_route.json').read_text())['forward_stages'][0]['samples'][0]
    seeds = [guide, {name: request['positions'][name] for name in planner.names}]
    hints = worker.json.loads((worker.EVIDENCE / 'inputs/elbow_seed_hints.json').read_text())
    seeds.append(hints['hints']['retrieval_pickup']['grip'])
    generator = np.random.default_rng(286)
    for unused in range(3):
        vector = np.clip(np.array([seeds[-1][name] for name in planner.names]) +
                         generator.normal(0, .7, len(planner.names)), planner.lower, planner.upper)
        seeds.append(dict(zip(planner.names, vector.tolist())))
    return seeds


def choose_elbow_pickup(request, worker):
    from minimum_turn import compact_qualified_approach, minimum_turn_connector, motion_metrics, prospective_transport
    if not active(request) or request['mode'] not in ('load_pickup', 'retrieval_pickup'):
        raise ValueError('Elbow pickup requires the declared empty-hand contract')
    if request.get('original_runtime_holder_references') != {}:
        raise ValueError('Cannot choose a different grasp while actually holding a box')
    started = time.monotonic()
    hints = worker.json.loads((worker.EVIDENCE / 'inputs/elbow_seed_hints.json').read_text())
    if hints['scope'] != 'PLANNING_HINTS_ONLY_REQUIRE_CURRENT_FULL_DENSE_FCL':
        raise ValueError('Invalid elbow planning hint scope')
    baseline = dict(stages=[dict(samples=hints['hints'][request['mode']]['approach'])])
    home = {name: request['positions'][name] for name in worker.TASK_JOINTS}
    candidates, diagnostics = [], []
    for flips in itertools.product((0, 180), repeat=2):
        references, targets = {}, {}
        for hand, flip in zip(('left', 'right'), flips):
            rotation = np.eye(4)
            rotation[:3, :3] = Rotation.from_euler('z', -flip, degrees=True).as_matrix()
            references[hand] = worker.nominal_references()[hand] @ rotation
            targets[hand] = worker.pose_matrix(request['poses']['box48_' + request['assignment'][hand]])
            if request.get('retrieval'):
                offset = request.get('retrieval_grip_height_offset_m', .010)
                if not 0 <= offset <= .020:
                    raise ValueError('Original retrieval height bound exceeded')
                targets[hand][2, 3] += offset
        planner = worker.PairedPlanner(request, references)
        unique = []
        for seed_index, seed in enumerate(pickup_seeds(worker, planner, request)):
            try:
                grip = planner.solve(targets, seed)
                if any(max(abs(grip[name] - prior[name]) for name in planner.names) < .05 for prior in unique):
                    continue
                unique.append(grip)
                if planner.planner.check(grip)['collision']:
                    continue
                metrics = posture_metrics(planner, grip)
                motion = motion_metrics(home, [grip], planner.names)
                candidates.append(dict(flips=list(flips), seed_index=seed_index, grip=grip,
                                       references=references, targets=targets,
                                       score=sum(row['from_down_deg'] for row in metrics.values()) + .2 * motion['score']))
            except (ValueError, RuntimeError) as error:
                diagnostics.append(dict(flips=list(flips), seed_index=seed_index, error=str(error)))
    candidates.sort(key=lambda value: value['score'])
    best = None
    for candidate in candidates[:6]:
        if best is not None and time.monotonic() - started > 85:
            break
        planner = worker.PairedPlanner(request, candidate['references'])
        try:
            above = copy.deepcopy(candidate['targets'])
            for body in above.values():
                body[2, 3] += .12
            pre = planner.solve(above, candidate['grip'])
            approach = minimum_turn_connector(planner, home, pre, baseline['stages'][0]['samples'])
            descent = planner.checked_stage(above, candidate['targets'], pre, 'descend', count=81)
            lookahead = prospective_transport(worker, request, planner, descent)
            evaluation = lookahead.get('elbow_posture', {})
            score = float(evaluation.get('basket_max_from_down_deg', 180))
            record = dict(status='PASS', stages=[dict(phase='approach', samples=approach), descent],
                          checked_samples=planner.checked, maximum_joint_step_rad=.002, planning_only_attachment=False,
                          assignment=request['assignment'], flips_deg=candidate['flips'], diagnostics=diagnostics,
                          exact_actual_full_state_used=True, no_physical_attachment=True,
                          prospective_retrieval_lookahead=lookahead if request.get('retrieval') else None,
                          prospective_loading_lookahead=lookahead if not request.get('retrieval') else None,
                          elbow_posture_selection=dict(contract=ELBOW_CONTRACT, basket_max_from_down_deg=score,
                              seed_index=candidate['seed_index'], observed_current_grasp_reference_not_rebased=True))
            if best is None or score < best['elbow_posture_selection']['basket_max_from_down_deg']:
                best = record
            if score < 45:
                break
        except (ValueError, RuntimeError) as error:
            diagnostics.append(dict(flips=candidate['flips'], seed_index=candidate['seed_index'], error=str(error)))
    if best is None:
        raise RuntimeError('No fully checked elbow-up pickup and carry: ' + worker.json.dumps(diagnostics))
    best['elbow_posture_selection']['elapsed_wall_s'] = time.monotonic() - started
    return best
