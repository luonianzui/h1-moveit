import copy
import itertools
import time

import numpy as np
from scipy.spatial.transform import Rotation
from bounded_connector import bounded_free_path


CONTRACT = 'current_pose_symmetric_grasp_v1'
CONTINUOUS_MODES = frozenset(('load_pickup', 'retrieval_pickup', 'pilot',
                              'load_transport', 'load_alignment', 'retrieval_transport'))
MAXIMUM_PATH_CANDIDATES = 8


def branch_seed(request, phase, seed, guides, index):
    marker = request.get('minimum_turn_contract')
    if marker is not None and marker != CONTRACT:
        raise ValueError('Unknown minimum-turn planning contract')
    if marker == CONTRACT and request.get('mode') in CONTINUOUS_MODES:
        return seed
    return seed if guides is None else guides[min(index, len(guides) - 1)]


def motion_metrics(first, points, names):
    names = sorted(names)
    if not points or any(set(point) != set(names) for point in points):
        raise ValueError('Minimum-turn score requires complete nonempty joint paths')
    matrix = np.array([[first[name] for name in names]] +
                      [[point[name] for name in names] for point in points], dtype=float)
    if not np.isfinite(matrix).all():
        raise ValueError('Nonfinite minimum-turn joint path')
    travel = np.abs(np.diff(matrix, axis=0)).sum(axis=0)
    axial = [index for index, name in enumerate(names)
             if any(suffix in name for suffix in ('shoulder_yaw', 'wrist_roll', 'wrist_yaw'))]
    excursion = np.ptp(matrix, axis=0)
    return dict(total_rad=float(travel.sum()), axial_rad=float(travel[axial].sum()),
                peak_excursion_rad=float(excursion.max()),
                score=float(travel.sum() + travel[axial].sum() + excursion.max()),
                joint_excursion_rad=dict(zip(names, excursion.tolist())))


def seed_candidates(planner, home, guide, legacy):
    candidates = [home, guide, legacy]
    for fraction in (.25, .5, .75):
        candidates.append({name: home[name] + fraction * (guide[name] - home[name])
                           for name in planner.names})
    for hand in ('left', 'right'):
        candidates.append({name: guide[name] if name.startswith(hand + '_') else home[name]
                           for name in planner.names})
    generator = np.random.default_rng(285)
    for unused in range(4):
        values = np.clip(np.array([home[name] for name in planner.names]) +
                         generator.normal(0, 1., len(planner.names)), planner.lower, planner.upper)
        candidates.append(dict(zip(planner.names, values)))
    return candidates


def direct_empty_connector(planner, first, last):
    names = planner.names
    origin = np.array([first[name] for name in names])
    goal = np.array([last[name] for name in names])
    count = max(2, int(np.ceil(np.max(np.abs(goal - origin)) / .002)) + 1)
    for vector in np.linspace(origin, goal, count):
        if np.any(vector < planner.lower - 1e-12) or np.any(vector > planner.upper + 1e-12):
            raise ValueError('Minimum-turn connector exceeds original joint margins')
        planner.checked += 1
        if planner.planner.check(dict(zip(names, vector.tolist())))['collision']:
            raise ValueError('Minimum-turn direct empty connector collision')
    samples = max(2, int(np.ceil(np.max(np.abs(goal - origin)) / .02)) + 1)
    return [dict(zip(names, vector.tolist())) for vector in np.linspace(origin, goal, samples)]


def minimum_turn_connector(planner, first, last, legacy_approach):
    best = None
    best_score = float('inf')
    for fraction in (0., .1, .2, .35, .5, .7):
        index = round(fraction * (len(legacy_approach) - 1))
        prefix = [dict(first)] + [dict(point) for point in legacy_approach[:index + 1]]
        try:
            for start, end in zip(prefix, prefix[1:]):
                direct_empty_connector(planner, start, end)
            tail = direct_empty_connector(planner, prefix[-1], last)
            path = prefix + tail[1:]
            score = motion_metrics(first, path, planner.names)['score']
            if score < best_score:
                best, best_score = path, score
        except ValueError:
            continue
    bound = motion_metrics(first, [last], planner.names)['score']
    if best is None or best_score > bound * 1.1:
        try:
            alternate = bounded_free_path(planner.planner, first, last)
            score = motion_metrics(first, alternate, planner.names)['score']
            if score < best_score:
                best, best_score = alternate, score
        except ValueError:
            pass
    if best is None:
        raise ValueError('No original-dense collision-free minimum-turn connector')
    return best


def compact_qualified_approach(planner, first, original):
    waypoints = [dict(first)] + [dict(point) for point in original]
    path = [dict(first)]
    index = 0
    while index < len(waypoints) - 1:
        remaining = len(waypoints) - 1 - index
        stride = max(1, remaining // 12)
        targets = list(range(len(waypoints) - 1, index, -stride))
        if index + 1 not in targets:
            targets.append(index + 1)
        for target in targets:
            try:
                connector = direct_empty_connector(planner, waypoints[index], waypoints[target])
                path.extend(connector[1:])
                index = target
                break
            except ValueError:
                continue
        else:
            raise ValueError('Original approach no longer valid in exact current scene')
    return path


def prospective_transport(worker, request, planner, descent):
    from elbow_posture import active, posture_metrics
    positions = dict(request['positions'], **descent['samples'][-1])
    planner.planner.state.joint_positions = positions
    planner.planner.state.update()
    references = {hand: (np.linalg.inv(planner.planner.state.get_global_link_transform(hand + '_wrist_yaw_link')) @
                         worker.pose_matrix(request['poses']['box48_' + payload])).tolist()
                  for hand, payload in request['assignment'].items()}
    pilot_request = dict(request, mode='pilot', positions=positions, planning_references=references)
    pilot = worker.plan_transport(pilot_request)
    positions.update(pilot['stages'][-1]['samples'][-1])
    planner.planner.state.joint_positions = positions
    planner.planner.state.update()
    poses = copy.deepcopy(request['poses'])
    for hand, payload in request['assignment'].items():
        body = planner.planner.state.get_global_link_transform(hand + '_wrist_yaw_link') @ np.array(references[hand])
        poses['box48_' + payload]['position'] = dict(zip('xyz', body[:3, 3].tolist()))
        poses['box48_' + payload]['orientation'] = dict(zip('xyzw', Rotation.from_matrix(body[:3, :3]).as_quat().tolist()))
    mode = 'retrieval_transport' if request['retrieval'] else 'load_transport'
    carry = worker.plan_transport(dict(request, mode=mode, positions=positions, poses=poses,
                                       planning_references=references))
    elbow_summary = None
    if active(request):
        summary = {hand: max(posture_metrics(planner, point)[hand]['from_down_deg'] for point in carry['stages'][0 if request['retrieval'] else -1]['samples']) for hand in ('left', 'right')}
        elbow_summary = dict(basket_max_from_down_deg=max(summary.values()), by_hand_from_down_deg=summary)
    return dict(status='PROSPECTIVE_RIGID_GEOMETRY_PASS_NOT_ACTUAL_GRASP',
                elbow_posture=elbow_summary,
                pilot_checks=pilot['checked_samples'], carry_checks=carry['checked_samples'],
                no_command_or_native_reference_freeze_from_prediction=True,
                actual_post_grasp_plan_and_guards_still_required=True)


def choose_pickup(request, worker):
    from elbow_posture import active, choose_elbow_pickup
    if active(request):
        return choose_elbow_pickup(request, worker)
    if request.get('minimum_turn_contract') != CONTRACT:
        return worker.legacy_plan_pickup(request)
    if request.get('mode') not in ('load_pickup', 'retrieval_pickup'):
        raise ValueError('Minimum-turn choice is only an empty pickup planner')
    if request.get('original_runtime_holder_references') != {}:
        raise ValueError('Minimum-turn choice cannot alter an actual held reference')
    began = time.monotonic()
    legacy_request = dict(request)
    legacy_request.pop('minimum_turn_contract')
    baseline = worker.legacy_plan_pickup(legacy_request)
    first = request['positions']
    baseline_metrics = motion_metrics(first, [point for stage in baseline['stages']
                                             for point in stage['samples']], worker.TASK_JOINTS)
    guide = worker.json.loads((worker.EVIDENCE / 'inputs/dual_route.json').read_text())['forward_stages'][0]['samples'][0]
    legacy_goal = baseline['stages'][-1]['samples'][-1]
    candidates, diagnostics = [], []
    for flips in itertools.product((0, 180), repeat=2):
        references, targets = {}, {}
        for hand, flip in zip(('left', 'right'), flips):
            rotation = np.eye(4)
            rotation[:3, :3] = Rotation.from_euler('z', -flip, degrees=True).as_matrix()
            references[hand] = worker.nominal_references()[hand] @ rotation
            targets[hand] = worker.pose_matrix(request['poses']['box48_' + request['assignment'][hand]])
            if request['mode'] == 'retrieval_pickup':
                offset = request.get('retrieval_grip_height_offset_m', .010)
                if not 0 <= offset <= .020:
                    raise ValueError('Original retrieval grip offset bound exceeded')
                targets[hand][2, 3] += offset
        planner = worker.PairedPlanner(request, references)
        unique = []
        for index, seed in enumerate(seed_candidates(planner, first, guide, legacy_goal)):
            try:
                solved = planner.solve(targets, seed)
                if any(max(abs(solved[name] - prior[name]) for name in planner.names) < .03 for prior in unique):
                    continue
                unique.append(solved)
                bound = motion_metrics(first, [solved], worker.TASK_JOINTS)
                if bound['score'] < baseline_metrics['score'] * .99 and bound['axial_rad'] < baseline_metrics['axial_rad'] * .99:
                    candidates.append(dict(flips=flips, seed_index=index, grip=solved,
                                           references=references, targets=targets, lower_bound=bound))
            except RuntimeError:
                pass
    best, best_metrics = baseline, baseline_metrics
    baseline_references = {}
    for hand, flip in zip(('left', 'right'), baseline['flips_deg']):
        rotation = np.eye(4)
        rotation[:3, :3] = Rotation.from_euler('z', -flip, degrees=True).as_matrix()
        baseline_references[hand] = worker.nominal_references()[hand] @ rotation
    try:
        planner = worker.PairedPlanner(request, baseline_references)
        compact = compact_qualified_approach(planner, {name: first[name] for name in planner.names},
                                              baseline['stages'][0]['samples'])
        stages = [dict(phase='approach', samples=compact)] + baseline['stages'][1:]
        metrics = motion_metrics(first, [point for stage in stages for point in stage['samples']], worker.TASK_JOINTS)
        if (metrics['score'] < baseline_metrics['score'] * .99 and
                metrics['axial_rad'] <= baseline_metrics['axial_rad'] + 1e-9 and
                metrics['peak_excursion_rad'] <= baseline_metrics['peak_excursion_rad'] + 1e-9):
            best = dict(baseline, stages=stages, checked_samples=baseline['checked_samples'] + planner.checked,
                        original_grasp_endpoint_and_descent_exactly_preserved=True)
            best_metrics = metrics
    except ValueError as error:
        diagnostics.append(dict(rejected='baseline approach compaction', error=str(error)))
    ordered = sorted(candidates, key=lambda item: item['lower_bound']['score'])
    diverse = []
    for limit in (1, 2):
        for candidate in ordered:
            if any(candidate is previous for previous in diverse):
                continue
            if sum(previous['flips'] == candidate['flips'] for previous in diverse) < limit:
                diverse.append(candidate)
    evaluated = 0
    for candidate in diverse[:MAXIMUM_PATH_CANDIDATES]:
        if time.monotonic() - began > 80.:
            break
        evaluated += 1
        try:
            planner = worker.PairedPlanner(request, candidate['references'])
            home = {name: first[name] for name in planner.names}
            above = copy.deepcopy(candidate['targets'])
            for body in above.values():
                body[2, 3] += .12
            pre = planner.solve(above, candidate['grip'])
            approach = minimum_turn_connector(planner, home, pre, baseline['stages'][0]['samples'])
            descent = planner.checked_stage(above, candidate['targets'], pre, 'descend', count=81)
            stages = [dict(phase='approach', samples=approach), descent]
            metrics = motion_metrics(first, [point for stage in stages for point in stage['samples']], worker.TASK_JOINTS)
            if (metrics['score'] >= best_metrics['score'] * .99 or
                    metrics['axial_rad'] >= best_metrics['axial_rad'] * .99 or
                    metrics['peak_excursion_rad'] > baseline_metrics['peak_excursion_rad'] + 1e-9):
                diagnostics.append(dict(flips_deg=list(candidate['flips']), seed_index=candidate['seed_index'],
                                        rejected='not a measured full-path improvement', metrics=metrics))
                continue
            lookahead = prospective_transport(worker, request, planner, descent)
            best_metrics = metrics
            best = dict(status='PASS', stages=stages, checked_samples=planner.checked,
                        maximum_joint_step_rad=.002, planning_only_attachment=False,
                        assignment=request['assignment'], flips_deg=list(candidate['flips']),
                        exact_actual_full_state_used=True, no_physical_attachment=True,
                        prospective_retrieval_lookahead=lookahead if request['retrieval'] else None,
                        prospective_loading_lookahead=lookahead if not request['retrieval'] else None)
        except (RuntimeError, ValueError) as error:
            diagnostics.append(dict(flips_deg=list(candidate['flips']), seed_index=candidate['seed_index'], error=str(error)))
    best = dict(best)
    best['minimum_turn_selection'] = dict(contract=CONTRACT, baseline=baseline_metrics,
        selected=best_metrics, improved=best_metrics['score'] < baseline_metrics['score'] * .99,
        exact_grasp_contact_frames_and_original_limits_preserved=True,
        candidate_paths_evaluated=evaluated, lower_bound_candidates=len(candidates),
        maximum_candidate_paths=MAXIMUM_PATH_CANDIDATES, diagnostics=diagnostics,
        selection_wall_s=time.monotonic() - began,
        old_rrt_fallback_preserved_not_all_solutions_claimed_optimal=True)
    return best
