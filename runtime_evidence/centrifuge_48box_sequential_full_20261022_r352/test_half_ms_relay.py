from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import pytest


SCRIPT = Path(__file__).with_name("inputs") / "robot_joint_state_relay_05ms.py"
SPEC = importlib.util.spec_from_file_location("joint_relay_r178", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
relay = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = relay
SPEC.loader.exec_module(relay)


ENTITY_ORDER = (
    *relay.LEG_JOINTS, "torso_joint", *relay.LEFT_ARM_JOINTS,
    *relay.LEFT_GRIPPER_JOINTS, *relay.RIGHT_ARM_JOINTS,
    *relay.RIGHT_GRIPPER_JOINTS,
)


def payload(names=ENTITY_ORDER):
    values = {name: float(index) / 100.0
              for index, name in enumerate(relay.CANONICAL_JOINTS)}
    return {
        "frame_id": "world",
        "names": names,
        "positions": tuple(values[name] for name in names),
        "velocities": tuple(values[name] + 1.0 for name in names),
        "efforts": tuple(values[name] + 2.0 for name in names),
    }


def accept_joint(core, stamp_ns, **overrides):
    value = payload()
    value.update(overrides)
    return core.accept_joint(stamp_ns=stamp_ns, **value)


def test_entity_order_is_reordered_without_changing_values():
    core = relay.RelayCore()
    accept_joint(core, 10_000_000)
    ready = core.accept_clock(10_000_000)
    assert len(ready) == 1
    sample = ready[0]
    assert sample.stamp_ns == 10_000_000
    assert sample.frame_id == "world"
    assert sample.positions == pytest.approx(
        tuple(float(index) / 100.0
              for index in range(len(relay.CANONICAL_JOINTS))))
    assert sample.velocities == pytest.approx(
        tuple(float(index) / 100.0 + 1.0
              for index in range(len(relay.CANONICAL_JOINTS))))
    assert sample.efforts == pytest.approx(
        tuple(float(index) / 100.0 + 2.0
              for index in range(len(relay.CANONICAL_JOINTS))))


def test_exact_half_ms_source_and_clock_publish_only_exact_10ms_grid():
    core = relay.RelayCore()
    published = []
    for millisecond in range(3, 46):
        stamp = millisecond * 500_000
        published.extend(accept_joint(core, stamp))
        published.extend(core.accept_clock(stamp))
    assert [item.stamp_ns for item in published] == [10_000_000, 20_000_000]
    assert core.source_samples == 43
    assert core.clock_samples == 43
    assert core.published_samples == 2
    assert core.pending == []


def test_clock_causally_gates_joint_publication():
    core = relay.RelayCore()
    assert accept_joint(core, 10_000_000) == ()
    assert accept_joint(core, 10_500_000) == ()
    assert core.accept_clock(9_500_000) == ()
    ready = core.accept_clock(10_000_000)
    assert [item.stamp_ns for item in ready] == [10_000_000]


@pytest.mark.parametrize("second", [10_000_000, 12_000_000, 9_000_000])
def test_duplicate_skipped_and_backward_joint_stamps_are_fatal(second):
    core = relay.RelayCore()
    accept_joint(core, 10_000_000)
    with pytest.raises(relay.RelayError, match="JOINT_SOURCE_STEP"):
        accept_joint(core, second)


@pytest.mark.parametrize("second", [10_000_000, 12_000_000, 9_000_000])
def test_duplicate_skipped_and_backward_clock_stamps_are_fatal(second):
    core = relay.RelayCore()
    core.accept_clock(10_000_000)
    with pytest.raises(relay.RelayError, match="CLOCK_SOURCE_STEP"):
        core.accept_clock(second)


def test_non_millisecond_stamp_is_fatal():
    core = relay.RelayCore()
    with pytest.raises(relay.RelayError, match="NOT_05MS_GRID"):
        accept_joint(core, 10_000_001)
    with pytest.raises(relay.RelayError, match="NOT_05MS_GRID"):
        core.accept_clock(10_000_001)


def test_name_set_must_be_exact_and_unique():
    core = relay.RelayCore()
    names = list(ENTITY_ORDER)
    names[-1] = names[0]
    with pytest.raises(relay.RelayError, match="JOINT_NAME_SET"):
        accept_joint(core, 10_000_000, names=names)


@pytest.mark.parametrize("field", ["positions", "velocities", "efforts"])
def test_all_four_joint_arrays_are_complete(field):
    core = relay.RelayCore()
    with pytest.raises(relay.RelayError, match="SIZE"):
        accept_joint(core, 10_000_000, **{field: (0.0,)})


@pytest.mark.parametrize("field", ["positions", "velocities", "efforts"])
def test_all_numeric_arrays_are_finite(field):
    core = relay.RelayCore()
    value = payload()[field]
    broken = (*value[:-1], math.nan)
    with pytest.raises(relay.RelayError, match="FINITE"):
        accept_joint(core, 10_000_000, **{field: broken})


def test_stamp_field_parser_requires_valid_exact_grid():
    assert relay._stamp_ns(12, 340_000_000, "TEST") == 12_340_000_000
    with pytest.raises(relay.RelayError, match="STAMP_FIELDS"):
        relay._stamp_ns(-1, 0, "TEST")
    with pytest.raises(relay.RelayError, match="NOT_05MS_GRID"):
        relay._stamp_ns(1, 1, "TEST")

