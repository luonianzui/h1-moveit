import copy
import inspect

import pytest

from contact_phase_policy import allow_planning_grasp_touch, bounded_motion
from dual_worker import EVIDENCE, plan_empty_return
from execute_dual import DualCycle


def test_old_identity_or_unqualified_guide_cannot_be_replayed():
    with pytest.raises(ValueError, match='current fresh run'):
        plan_empty_return(dict(current_run_guide_identity='old_run'))
    with pytest.raises(ValueError, match='qualified current-run'):
        plan_empty_return(dict(current_run_guide_identity=EVIDENCE.name))


def test_new_reverse_near_basket_phase_is_bounded_without_relaxing_collision_guards():
    assert bounded_motion('machine_paired_empty_reverse_forward_inside_basket_raise', .8) == (.13, .002)
    assert allow_planning_grasp_touch('empty_return') is False


def test_guides_are_only_qualified_after_actual_drop_and_include_optional_alignment():
    source = inspect.getsource(DualCycle.load_pair)
    assert source.index('self.observe_pair_drop()') < source.index('self.qualified_empty_retreat_guides = retreat_guides')
    assert "retreat_guides.extend(copy.deepcopy(correction['stages']))" in source
    assert 'both_actual_drops_passed=True' in source
