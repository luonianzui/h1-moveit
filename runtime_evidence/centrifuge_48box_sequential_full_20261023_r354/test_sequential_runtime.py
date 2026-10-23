import copy
from types import SimpleNamespace

import pytest

from dual_motion import HoldRegistry
from endpoint_overlap_cycle import EndpointOverlapCycle
from sequential_cycle import SequentialCycle
from sequential_order import SequentialOrder


def pose(payload):
    return dict(position=dict(x=.44, y=.38 if payload == 'left' else -.38, z=.84475), orientation=dict(w=1.))


def table_pair(payload):
    return dict(first='box48_' + payload + '::body::collision', second='table::surface::collision',
                force=1., depth=0., normals=[], force_vectors=[])


def guard_fixture(monkeypatch):
    monkeypatch.setattr(EndpointOverlapCycle, 'safety', lambda self: None)
    cycle = SequentialCycle.__new__(SequentialCycle)
    cycle.holders = HoldRegistry()
    cycle.order = SequentialOrder()
    cycle.remaining_guard_enabled = True
    cycle.release_transition = None
    cycle.table_references = {}
    cycle.table_positive = {}
    cycle.remaining_last_record = -1.
    cycle.sim = cycle.pose_time = 1.
    cycle.phase = 'test_only'
    cycle.poses = {'box48_' + payload: pose(payload) for payload in ('left', 'right')}
    cycle.contacts = {payload: [table_pair(payload)] for payload in ('left', 'right')}
    cycle.return_hands = {}
    cycle.return_table_yaws = {}
    cycle.record = lambda *args: None
    return cycle


def test_remaining_table_monitor_accepts_both_original_supported_births(monkeypatch):
    cycle = guard_fixture(monkeypatch)
    cycle.safety()
    assert set(cycle.table_positive) == {'left', 'right'}


def test_second_actual_holder_is_rejected_before_any_motion(monkeypatch):
    cycle = guard_fixture(monkeypatch)
    cycle.holders.rows = {'left': dict(payload='left'), 'right': dict(payload='right')}
    with pytest.raises(RuntimeError, match='two actual holders'):
        cycle.safety()


def test_unheld_other_table_box_cannot_drift_during_pickup(monkeypatch):
    cycle = guard_fixture(monkeypatch)
    cycle.holders.rows = {'left': dict(payload='left')}
    cycle.order.locations['left'] = 'held'
    cycle.poses['box48_right']['position']['x'] += .00031
    with pytest.raises(RuntimeError, match='birth pose'):
        cycle.safety()


def test_unheld_other_table_box_cannot_lose_support(monkeypatch):
    cycle = guard_fixture(monkeypatch)
    cycle.contacts['right'] = []
    with pytest.raises(RuntimeError, match='support/pose/contact'):
        cycle.safety()


def test_finger_contact_requires_explicit_actual_grasp_transition(monkeypatch):
    cycle = guard_fixture(monkeypatch)
    cycle.contacts['left'].append(dict(table_pair('left'), second='h1_2::left_dex1_finger_1_link::collision'))
    with pytest.raises(RuntimeError, match='support/pose/contact'):
        cycle.safety()
    cycle.grasp_candidate, cycle.current_grasp_hand = 'left', 'left'
    cycle.safety()


def test_grasp_transition_does_not_ignore_other_hand_or_robot_contact(monkeypatch):
    cycle = guard_fixture(monkeypatch)
    cycle.grasp_candidate, cycle.current_grasp_hand = 'left', 'left'
    cycle.contacts['left'].append(dict(table_pair('left'), second='h1_2::right_dex1_finger_1_link::collision'))
    with pytest.raises(RuntimeError, match='support/pose/contact'):
        cycle.safety()


def test_actual_holder_cannot_disappear_without_qualified_release(monkeypatch):
    cycle = guard_fixture(monkeypatch)
    cycle.order.locations['left'] = 'held'
    with pytest.raises(RuntimeError, match='holder disappearance'):
        cycle.safety()


def test_release_transition_has_bounded_deadline(monkeypatch):
    cycle = guard_fixture(monkeypatch)
    cycle.order.locations['left'] = 'held'
    cycle.release_transition = dict(payload='left', deadline_sim=1.1)
    cycle.safety()
    cycle.sim = 1.101
    with pytest.raises(RuntimeError, match='observation window'):
        cycle.safety()


@pytest.mark.parametrize('phase,held,expected', [('seq_load_empty_approach', False, (.35, .015)),
    ('seq_load_load_transfer', True, (.25, .015)), ('seq_load_load_lower', True, (.13, .002)),
    ('seq_retrieval_single_pilot', True, (.03, .002)), ('paired_retrieval_setdown_fine', True, (.03, .002)),
    ('seq_load_empty_vertical_retreat', False, (.13, .002))])
def test_single_phase_limits_keep_original_precision(phase, held, expected):
    cycle = SimpleNamespace(holders=SimpleNamespace(rows={'left': dict(payload='left')} if held else {}))
    assert SequentialCycle.motion_limits(cycle, phase, .8) == expected


def test_remaining_basket_is_not_ignored_while_other_hand_holds(monkeypatch):
    import sequential_cycle
    from support_continuity import SupportContinuity
    cycle = guard_fixture(monkeypatch)
    cycle.order.locations = dict(left='basket', right='held')
    cycle.holders.rows = {'right': dict(payload='right')}
    cycle.support_continuity = dict(left=SupportContinuity(.02))
    monkeypatch.setattr(sequential_cycle, 'retention_metrics', lambda *args: dict(supported=True,
        contacts_valid=True, sidewall_certified=False, vertical_inside=True))
    with pytest.raises(RuntimeError, match='remaining box'):
        cycle.safety()


@pytest.mark.parametrize('own_hand,payload', [('left', 'right'), ('right', 'left')])
def test_current_retrieval_target_allows_own_preload_but_retains_basket_support(monkeypatch, own_hand, payload):
    import sequential_cycle
    from support_continuity import SupportContinuity
    cycle = guard_fixture(monkeypatch)
    cycle.order.locations = dict(left='basket', right='basket')
    cycle.support_continuity = {side: SupportContinuity(.02) for side in ('left', 'right')}
    cycle.phase = own_hand + '_retrieval_grasp'
    cycle.grasp_candidate, cycle.current_grasp_hand = payload, own_hand
    for side in ('left', 'right'):
        cycle.contacts[side] = [dict(table_pair(side), second='centrifuge::bucket_' + side + '::collision')]
    cycle.contacts[payload].append(dict(table_pair(payload), second='h1_2::' + own_hand + '_dex1_finger_1_pad_link'))
    observed = []

    def retention(poses, pairs, side):
        observed.append((side, copy.deepcopy(pairs)))
        return dict(supported=bool(pairs), contacts_valid=True, sidewall_certified=True, vertical_inside=True)

    monkeypatch.setattr(sequential_cycle, 'retention_metrics', retention)
    cycle.safety()
    assert all(not any('dex1' in pair['second'] for pair in pairs) for side, pairs in observed)
    assert all(len(pairs) == 1 for side, pairs in observed)
    other = 'left' if payload == 'right' else 'right'
    cycle.contacts[other].append(dict(table_pair(other), second='h1_2::' + own_hand + '_dex1_finger_1_pad_link'))
    with pytest.raises(RuntimeError, match='remaining box'):
        cycle.safety()


def test_current_retrieval_grasp_cannot_replace_basket_support_with_finger_force(monkeypatch):
    import sequential_cycle
    from support_continuity import SupportContinuity
    cycle = guard_fixture(monkeypatch)
    cycle.order.locations = dict(left='basket', right='basket')
    cycle.support_continuity = {side: SupportContinuity(.02) for side in ('left', 'right')}
    cycle.phase, cycle.grasp_candidate, cycle.current_grasp_hand = 'left_retrieval_grasp', 'left', 'left'
    cycle.contacts['left'] = [dict(table_pair('left'), second='h1_2::left_dex1_finger_1_pad_link')]
    monkeypatch.setattr(sequential_cycle, 'retention_metrics', lambda poses, pairs, side: dict(
        supported=bool(pairs), contacts_valid=True, sidewall_certified=True, vertical_inside=True))
    with pytest.raises(RuntimeError, match='remaining box'):
        cycle.safety()


def test_loading_orchestration_never_captures_two_boxes_together(monkeypatch):
    cycle = SimpleNamespace(order=SequentialOrder(), holders=HoldRegistry(), sim=1., max_native_slip=0.,
        contacts={payload: [table_pair(payload)] for payload in ('left', 'right')},
        poses={'box48_' + payload: pose(payload) for payload in ('left', 'right')})
    timeline = []
    def alignment(current, hand, payload):
        assert set(current.holders.rows) == {hand}
        assert current.holders.rows[hand]['payload'] == payload
        timeline.append(('alignment', hand, payload))
    monkeypatch.setattr('sequential_cycle.align_before_release', alignment)
    cycle.record = lambda *args: None
    cycle.single_gripper = lambda hand, target, effort, phase: timeline.append(('gripper', hand, effort, tuple(cycle.holders.rows)))
    cycle.single_plan = lambda mode, hand, payload: dict(mode=mode, hand=hand, payload=payload)

    def execute(planned, prefix):
        timeline.append(('plan', planned['mode'], planned['hand']))
        if planned['mode'] == 'load_pilot':
            cycle.poses['box48_' + planned['payload']]['position']['z'] = .85975
            cycle.contacts[planned['payload']] = []

    def capture(assignment):
        assert not cycle.holders.rows and len(assignment) == 1
        hand, payload = next(iter(assignment.items()))
        cycle.holders.rows = {hand: dict(payload=payload, source_sim=cycle.sim,
            native_reference=SimpleNamespace(tolist=lambda: []), fk_reference=SimpleNamespace(tolist=lambda: []))}
        timeline.append(('capture', hand, payload))

    def release(reason):
        assert len(cycle.holders.rows) == 1
        timeline.append(('release', next(iter(cycle.holders.rows))))
        cycle.holders.rows.clear()

    def dwell(seconds):
        cycle.sim += seconds

    cycle.execute_stages, cycle.capture_pair, cycle.forget_released_pair = execute, capture, release
    cycle.dwell, cycle.check_pair_release = dwell, lambda: None
    cycle.observe_drop = lambda payload: 3.
    SequentialCycle.load_pair(cycle)
    assert cycle.order.loaded == ['left', 'right'] and cycle.order.phase == 'machine'
    assert [row for row in timeline if row[0] in ('capture', 'release')] == [
        ('capture', 'left', 'left'), ('release', 'left'), ('capture', 'right', 'right'), ('release', 'right')]
    for hand in ('left', 'right'):
        assert timeline.index(('capture', hand, hand)) < timeline.index(('alignment', hand, hand)) < timeline.index(('release', hand))


def test_empty_endpoint_uses_actual_native_feedback_not_command_value():
    import ast
    from pathlib import Path
    source = (Path(__file__).resolve().parent / 'sequential_cycle.py').read_text()
    tree = ast.parse(source)
    method = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == 'single_gripper')
    text = ast.get_source_segment(source, method)
    assert 'wait_original_gripper_endpoint(self, hand, target, self.sim)' in text
    assert 'response.result.error_code != 0' in text
    assert 'profile = retrieval_profile(' in text
    from sequential_force import retrieval_profile
    profile = retrieval_profile([-.0199, -.0199], -.01901)
    assert len(profile) == 61 and profile[-1]['effort'] == [16.5, 16.5]


@pytest.mark.parametrize('name', ['run_guard', 'disk_watch'])
def test_owned_sigint_guard_is_rebound_only_to_new_executor(name):
    from pathlib import Path
    root = Path(__file__).resolve().parent
    original = (root / (name + '.py')).read_text()
    wrapper = (root / ('sequential_' + name + '.py')).read_text()
    assert original.count("'execute_cycle.py'") == 2
    assert 'source.replace("\'execute_cycle.py\'", "\'execute_sequential.py\'")' in wrapper
    new = original.replace("'execute_cycle.py'", "'execute_sequential.py'")
    assert 'signal.SIGINT' in new and 'SIGKILL' not in new
    launch = (root / 'launch_run.py').read_text()
    assert "'sequential_" + name + ".py'" in launch


def test_new_entry_preserves_old_fresh_health_claim_and_failure_lifecycle():
    from pathlib import Path
    root = Path(__file__).resolve().parent
    source = (root / 'execute_sequential.py').read_text()
    for marker in ('verify_interrupt_delivery(os.getpid())', 'read_contract(EVIDENCE)',
                   'verify_manager_rate_output(', 'verify_gain_output(', 'verify_damping_output(',
                   'require_spline_output(', 'validate_dispatch(EVIDENCE)', "'run/execution.claim'",
                   'cycle.hold(exception)', 'cycle.close()', 'rclpy.try_shutdown()'):
        assert marker in source
    assert "identity['maximum_actual_holders'] == 1" in source
    assert 'paired_actual_cycle=False, sequential_actual_cycle=True' in source
