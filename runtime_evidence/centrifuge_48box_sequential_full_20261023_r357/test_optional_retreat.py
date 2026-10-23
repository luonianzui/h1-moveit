import copy
import json
from pathlib import Path

import pytest

from gripper_return_geometry import FINGERS, verify_request
from post_retreat_contract import RETREAT_PHASES, expected_retreat_phases
from post_retreat_gripper_cycle import PostRetreatGripperCycle
from test_retrieval_grip import candidate
from test_complete_rate_profile import actual_publisher_cycle


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_bucket_axis_exit_full_20261015_r282'


def actual():
    request = json.loads((PARENT/'run/5_empty_return_request.json').read_text())
    planned = json.loads((PARENT/'run/5_empty_return_result.json').read_text())
    return request,planned


def test_exact_actual_optional_alignment_is_not_skipped_and_all_original_stages_remain_required():
    request,planned = actual()
    expected = expected_retreat_phases(request['current_run_executed_loading_guides'])
    assert expected == (RETREAT_PHASES[0],) + RETREAT_PHASES
    assert tuple(row['phase'] for row in planned['stages']) == expected + ('empty_return',)
    assert expected_retreat_phases(request['current_run_executed_loading_guides'][:-1]) == RETREAT_PHASES
    assert expected_retreat_phases() == RETREAT_PHASES


@pytest.mark.parametrize('invalid',['unknown_phase','reorder','extra_alignment','missing_stage','incomplete','nonfinite'])
def test_unreviewed_incomplete_nonfinite_or_multiple_optional_corrections_reject(invalid):
    request,planned = actual()
    guides = request['current_run_executed_loading_guides']
    if invalid == 'unknown_phase':
        guides[-1]['phase'] = 'unreviewed'
    elif invalid == 'reorder':
        guides[1],guides[2] = guides[2],guides[1]
    elif invalid == 'extra_alignment':
        guides.append(copy.deepcopy(guides[-1]))
    elif invalid == 'missing_stage':
        guides.pop(1)
    elif invalid == 'incomplete':
        guides[-1]['samples'][0].pop('torso_joint')
    else:
        guides[-1]['samples'][0]['torso_joint'] = float('nan')
    with pytest.raises(ValueError):
        expected_retreat_phases(guides)


def test_actual_bilateral_ros_gripper_goals_only_after_all_four_reverse_stages_and_both_geometry_checks():
    request,planned = actual()
    cycle,goals = candidate()
    cycle.__class__ = PostRetreatGripperCycle
    cycle.qualified_empty_retreat_guides = request['current_run_executed_loading_guides']
    calls = []
    expected = expected_retreat_phases(cycle.qualified_empty_retreat_guides)
    def plan(mode,assignment):
        calls.append(('plan',mode))
        if mode == 'empty_return':
            return planned
        assert tuple(cycle.completed_empty_retreat_stages) == expected
        if mode == 'empty_gripper_after_retreat':
            assert not goals
            return dict(status='PASS')
        assert set(goals) == {'left','right'}
        assert cycle.positions == dict.fromkeys(cycle.positions,-.019)
        return dict(stages=[dict(phase='empty_return',samples=[dict(fresh=True)])])
    cycle.actual_plan = plan
    cycle.move = lambda samples,phase,rate: calls.append(('move',phase,rate))
    cycle.return_neutral()
    assert [row for row in calls if row[0]=='move'] == [
        ('move','machine_paired_'+phase,.13 if phase==RETREAT_PHASES[0] else .8) for phase in expected] + [
        ('move','machine_paired_empty_return',.8)]
    assert cycle.motion_count == 2
    assert cycle.accepted_gripper_efforts == dict(left=0.,right=0.)
    assert len([row for row in cycle.events if row[0]=='original_gripper_endpoint_fresh_feedback_pass']) == 2


@pytest.mark.parametrize('missing',['alignment','outside','identity','effort'])
def test_worker_needs_exact_actual_completed_sequence_current_identity_and_zero_effort(missing):
    source,planned = actual()
    request = dict(mode='empty_gripper_after_retreat',current_run_guide_identity='fresh',
        original_runtime_holder_references={},accepted_gripper_efforts=dict(left=0.,right=0.),
        source_sim=70.,positions=dict.fromkeys(FINGERS,-.0199),
        current_run_executed_loading_guides=source['current_run_executed_loading_guides'],
        completed_empty_retreat_stages=list(expected_retreat_phases(source['current_run_executed_loading_guides'])))
    if missing == 'alignment':
        request['completed_empty_retreat_stages'].pop(0)
    elif missing == 'outside':
        request['completed_empty_retreat_stages'].pop()
    elif missing == 'identity':
        request['current_run_guide_identity'] = 'old'
    else:
        request['accepted_gripper_efforts']['right'] = 16.
    with pytest.raises(ValueError):
        verify_request(request,'fresh')


def test_both_optional_and_original_inside_basket_reverse_use_original_point_thirteen_three_ros_dispatch():
    request,planned = actual()
    cycle = actual_publisher_cycle(loaded=False)
    cycle.__class__ = PostRetreatGripperCycle
    cycle.positions.update(request['positions'])
    for stage in planned['stages'][:2]:
        assert stage['phase'] == RETREAT_PHASES[0]
        cycle.move(stage['samples'],'machine_paired_'+stage['phase'],.13)
    commands = [value for kind,value in cycle.events if kind=='command']
    assert len(commands) == 2 and all(row['actual_rate_rad_s']==.13 for row in commands)
    assert all(len(publisher.messages)==2 for publisher in cycle.publishers.values())
    assert cycle.motion_count == 6
