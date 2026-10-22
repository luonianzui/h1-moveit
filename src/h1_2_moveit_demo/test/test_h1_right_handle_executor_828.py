"""Offline fail-closed tests for the sealed 828 right-handle executor."""

import ast
import builtins
import hashlib
import importlib.util
import json
import math
import sys
import types
from pathlib import Path

import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
SOURCE = WORKSPACE / "src/h1_2_moveit_demo/scripts/h1_right_handle_executor_828.py"
SPEC = importlib.util.spec_from_file_location("h1_right_handle_executor_828_test", SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _sealed_json(path, value):
    path.write_text(json.dumps(value, sort_keys=True) + "\n", encoding="utf-8")
    path.chmod(0o400)
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _sealed_bytes(path, value):
    path.write_bytes(value)
    path.chmod(0o400)
    return hashlib.sha256(value).hexdigest()


def _fixture(tmp_path, *, controller_authorized=True, torso=0.65,
             path_schema=None):
    arm = list(MODULE.NEUTRAL_RIGHT_ARM)
    rows = []
    for index, angle in enumerate((0.0, 90.0)):
        row_arm = arm if index == 0 else list(MODULE.Q90_MIDDLE_SEAM_RIGHT_ARM)
        row_torso_deg = (math.degrees(torso) if index == 0 else
                          math.degrees(MODULE.Q90_MIDDLE_SEAM_TORSO_RAD))
        row_gripper_mm = ((15.3, 15.3) if index == 0 else
                          tuple(value * 1000.0 for value in
                                MODULE.Q90_MIDDLE_SEAM_RIGHT_GRIPPER))
        rows.append({
            "index": index,
            "handle_angle_deg": angle,
            "contact_index": index,
            "action": "contact_arc_right_pad1_load_bearing",
            "geometry_source": "test/v1",
            "torso_yaw_deg": row_torso_deg,
            "right_arm_rad": row_arm,
            "phase_deg": -180.0,
            "z_mm": 95.0,
            "palm_down_tilt_deg": 7.0,
            "side_tilt_deg": 0.0,
            "outward_offset_mm": 16.0,
            "local_x_offset_mm": 0.0,
            "right_pad1_slider_mm": row_gripper_mm[0],
            "right_pad2_slider_mm": row_gripper_mm[1],
            "original_stl_gate": {
                "pass_": True,
                "pad1_contact_authorized": True,
                "nonpad_contact_forbidden": True,
                "left_hand_handle_contact_forbidden": True,
            },
        })
    path_value = {
        "$schema": path_schema or MODULE.PATH_SCHEMA,
        "schema_version": 4,
        "execution_topology": MODULE.EXECUTION_TOPOLOGY,
        "status": {
            "execution_ready": False,
            "gazebo_slow_contact_dynamics_complete": False,
            "offline_kinematic_path_valid": False,
            "forward_discrete_path_valid": False,
            "reverse_discrete_path_valid": False,
            "exact_q90_endpoint_present": True,
            "canonical_e0_q90_endpoint_present": True,
            "torso_variable": True,
            "forward_dense_exact_complete": False,
            "reverse_dense_exact_complete": False,
            "full_execution_topology_exact_complete": False,
            "ik_retarget_complete": True,
            "fail_closed": True,
            "reason": "external exact audit required",
        },
        "contract": {
            "all_nonpad_handle_contact_forbidden": True,
            "authorized_load_contact": "right_pad1",
            "handle_end_deg": 90.0,
            "handle_start_deg": 0.0,
            "left_hand_handle_contact_forbidden": True,
            "maximum_adjacent_joint_step_deg": 4.9,
            "maximum_dense_contact_geometry_angular_step_deg": 0.1,
            "maximum_dense_contact_geometry_linear_step_mm": 0.1,
            "maximum_dense_slider_step_mm": 0.1,
            "middle_initial": "q90_continuous_grasp",
            "middle_terminal": "q90_continuous_grasp",
            "minimum_arm_joint_limit_margin_deg": 2.0,
            "opposite_right_pad_contact_permitted": True,
            "partial_path_forbidden": True,
            "prefix_terminal": "q90_continuous_grasp",
            "q90_regrasp_commands": 0,
            "q90_release_commands": 0,
            "suffix_initial": "q90_continuous_grasp",
            "suffix_release_only_after_q0": True,
            "torso_abs_limit_deg": 44.5,
        },
        "metrics": {
            "maximum_abs_torso_deg": max(
                abs(math.degrees(torso)),
                abs(math.degrees(MODULE.Q90_MIDDLE_SEAM_TORSO_RAD))),
            "maximum_adjacent_8dof_step_deg": 1.0,
            "maximum_tcp_orientation_retarget_error_deg": 0.0,
            "maximum_tcp_position_retarget_error_mm": 0.0,
            "minimum_arm_margin_deg_discrete": 3.0,
            "retargeted_row_count": 0,
            "row_count": len(rows),
        },
        "rows": rows,
    }
    path_path = tmp_path / "path.json"
    path_sha = _sealed_json(path_path, path_value)
    scene_path = tmp_path / "scene.cdr"
    scene_sha = _sealed_bytes(scene_path, b"sealed-scene")
    q0 = (torso, *MODULE.NEUTRAL_RIGHT_ARM)
    neutral = (0.0, *MODULE.NEUTRAL_RIGHT_ARM)
    waypoints = [
        [first + (second - first) * fraction
         for first, second in zip(neutral, q0)]
        for fraction in (0.0, 0.35, 0.70, 1.0)
    ]
    ingress_value = {
        "schema": MODULE.INGRESS_SCHEMA,
        "status": "PASS",
        "valid": True,
        "controller_commands_issued": 0,
        "offline_only": True,
        "path_sha256": MODULE.EXPECTED_INGRESS_SOURCE_PATH_SHA256,
        "scene_cdr_sha256": scene_sha,
        "handle_angle_deg": 0,
        "park_left_shoulder_roll_rad": 0.316,
        "goal_right_arm_rad": list(MODULE.NEUTRAL_RIGHT_ARM),
        "strict_revalidation_step_rad": MODULE.HARD_DENSE_ANGULAR_STEP_RAD,
        "waypoint_count": len(waypoints),
        "configuration_order": ["torso_joint", *MODULE.RIGHT_ARM_JOINTS],
        "right_fingers_rad_or_m": {
            name: -0.019 for name in MODULE.RIGHT_GRIPPER_JOINTS
        },
        "waypoints": waypoints,
    }
    ingress_path = tmp_path / "ingress.json"
    ingress_sha = _sealed_json(ingress_path, ingress_value)
    prefix_samples = {name: 1 for name in MODULE.AUDITED_PREFIX_STAGES}
    suffix_samples = {name: 1 for name in MODULE.AUDITED_SUFFIX_STAGES}
    audit_value = {
        "schema": MODULE.AUDIT_SCHEMA,
        "auditor_version": MODULE.EXPECTED_AUDITOR,
        "execution_topology": MODULE.EXECUTION_TOPOLOGY,
        "valid": True,
        "status": "PASS",
        "controller_commands_issued": 0,
        "offline_only": True,
        "path_sha256": path_sha,
        "q0_ingress_asset_sha256": ingress_sha,
        "q0_ingress_waypoint_count": len(waypoints),
        "raw_scene_cdr_sha256": scene_sha,
        "prefix_passed": True,
        "suffix_passed": True,
        "execution_splices_included": True,
        "q90_continuous_grasp_seam_passed": True,
        "q90_release_commands": 0,
        "q90_regrasp_commands": 0,
        "forward_passed": True,
        "reverse_passed": True,
        "q0_ingress_forward_passed": True,
        "q0_ingress_exact_reverse_egress_passed": True,
        "variable_torso_path": True,
        "collision_free": True,
        "self_collision_checked": True,
        "full_world_and_table_guard_checked": True,
        "original_handle_stl_sha256": MODULE.EXPECTED_HANDLE_STL_SHA256,
        "minimum_nonpad_handle_clearance_m": 0.00061,
        "minimum_left_handle_clearance_m": 0.01001,
        "maximum_abs_torso_rad": max(
            abs(torso), abs(MODULE.Q90_MIDDLE_SEAM_TORSO_RAD)),
        "torso_abs_contract_rad": MODULE.HARD_MAX_TORSO_RAD,
        "dense_limits": {
            "angular_rad": MODULE.HARD_DENSE_ANGULAR_STEP_RAD,
            "prismatic_m": MODULE.HARD_DENSE_PRISMATIC_STEP_M,
        },
        "left_handle_allowed_contact_pairs": [],
        "authorized_handle_contact_pairs": [
            ["incubator_handle_exact", "right_dex1_finger_1_pad_link"],
            ["incubator_handle_exact", "right_dex1_finger_2_pad_link"],
        ],
        "directions": [
            {"name": "prefix", "stage_samples": prefix_samples},
            {"name": "suffix", "stage_samples": suffix_samples},
        ],
        "execution_contract": {
            "prefix_stages": [
                "prepare_left_park", "ingress_open_to_q0", "close_q0",
                "contact_q0_to_q90_hold",
            ],
            "suffix_stages": [
                "q90_continuous_grasp_seam",
                "contact_reverse_q90_to_q0", "release_q0",
                "egress_open_from_q0", "return_left",
            ],
            "q90_release_forbidden": True,
            "q90_regrasp_forbidden": True,
            "q0_ingress_fingers_open_m": -0.019,
        },
        "launch_left_shoulder_roll_rad": MODULE.LAUNCH_LEFT_ARM[1],
        "park_left_shoulder_roll_rad": 0.316,
    }
    audit_path = tmp_path / "audit.json"
    audit_sha = _sealed_json(audit_path, audit_value)
    manifest_value = {
        "schema": MODULE.MANIFEST_SCHEMA,
        "controller_execution_authorized": controller_authorized,
        "execution_ready": False,
        "live_execution_passed": False,
        "live_result": None,
        "controller_commands_issued": 0,
        "offline_only": True,
        "auditor_version": MODULE.EXPECTED_AUDITOR,
        "execution_topology": MODULE.EXECUTION_TOPOLOGY,
        "bindings": {
            "contact_path": {"path": path_path.name, "sha256": path_sha},
            "ingress_path": {"path": ingress_path.name, "sha256": ingress_sha},
            "exact_audit": {"path": audit_path.name, "sha256": audit_sha},
            "scene_snapshot": {"path": scene_path.name, "sha256": scene_sha},
        },
        "controller_topics": list(MODULE.CANONICAL_CONTROLLER_TOPICS),
        "stage_order": {
            "prefix": list(MODULE.PREFIX_STAGE_ORDER),
            "suffix": list(MODULE.SUFFIX_STAGE_ORDER),
        },
        "limits": {
            "maximum_abs_torso_rad": max(
                MODULE.HARD_MAX_TORSO_RAD, abs(torso)),
            "dense_angular_step_rad": MODULE.HARD_DENSE_ANGULAR_STEP_RAD,
            "dense_prismatic_step_m": MODULE.HARD_DENSE_PRISMATIC_STEP_M,
            "start_tolerance_rad": 0.003,
            "endpoint_tolerance_rad": 0.003,
            "gripper_tolerance_m": 0.001,
            "lead_time_s": 0.25,
            "point_dt_s": 0.02,
        },
    }
    manifest_path = tmp_path / "manifest.json"
    manifest_sha = _sealed_json(manifest_path, manifest_value)
    # Production pins the real 826.3 registry hashes.  Synthetic unit-test
    # documents replace only those registry identities; all semantic gates
    # remain active.
    MODULE.EXPECTED_PATH_SHA256 = path_sha
    MODULE.EXPECTED_INGRESS_SHA256 = ingress_sha
    MODULE.EXPECTED_AUDIT_SHA256 = audit_sha
    return manifest_path, manifest_sha


def test_valid_bundle_and_both_stage_orders_are_ros_free(tmp_path, monkeypatch):
    manifest, digest = _fixture(tmp_path)
    original_import = builtins.__import__

    def guarded_import(name, *args, **kwargs):
        if name == "rclpy" or name.startswith("rclpy."):
            raise AssertionError("validation imported ROS")
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded_import)
    bundle = MODULE.validate_bundle(tmp_path, manifest, digest)
    prefix = MODULE.build_stages(bundle, "prefix")
    suffix = MODULE.build_stages(bundle, "suffix")
    assert tuple(stage.name for stage in prefix) == MODULE.PREFIX_STAGE_ORDER
    assert tuple(stage.name for stage in suffix) == MODULE.SUFFIX_STAGE_ORDER
    assert prefix[0].states[0]["left_shoulder_roll_joint"] == pytest.approx(
        MODULE.LAUNCH_LEFT_ARM[1])
    assert prefix[0].states[-1]["left_shoulder_roll_joint"] == 0.316
    assert suffix[-1].states[-1]["left_shoulder_roll_joint"] == pytest.approx(
        MODULE.LAUNCH_LEFT_ARM[1])
    assert prefix[-1].name == "contact_arc_q0_to_q90_hold"
    assert prefix[-1].states[-1]["right_dex1_finger_1_joint"] == pytest.approx(0.0108)
    assert suffix[0].name == "contact_arc_q90_hold_to_q0"
    assert suffix[0].states[0]["right_dex1_finger_1_joint"] == pytest.approx(0.0108)
    assert all("release_q90" not in stage.name for stage in prefix)


def test_current_hardened_candidate_manifest_is_rejected_before_schema():
    path = (WORKSPACE / "runtime_assets/right_handle_8263_natural/"
            "hardened_splice_candidates_828/manifest.json")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    with pytest.raises(
            MODULE.ExecutionError,
            match="MANIFEST_NOT_CONTROLLER_AUTHORIZED"):
        MODULE.validate_bundle(WORKSPACE, path, digest)


def test_false_controller_authorization_is_the_first_gate(tmp_path):
    manifest, digest = _fixture(tmp_path, controller_authorized=False)
    value = json.loads(manifest.read_text())
    value["schema"] = "untrusted/schema"
    manifest.chmod(0o600)
    digest = _sealed_json(manifest, value)
    with pytest.raises(
            MODULE.ExecutionError,
            match="MANIFEST_NOT_CONTROLLER_AUTHORIZED"):
        MODULE.validate_bundle(tmp_path, manifest, digest)


def test_authorization_manifest_cannot_pretend_live_success(tmp_path):
    manifest, _ = _fixture(tmp_path)
    value = json.loads(manifest.read_text())
    value["execution_ready"] = True
    value["live_execution_passed"] = True
    value["live_result"] = {"sha256": "0" * 64}
    manifest.chmod(0o600)
    digest = _sealed_json(manifest, value)
    with pytest.raises(MODULE.ExecutionError, match="MANIFEST_PRE_RUN_STATE"):
        MODULE.validate_bundle(tmp_path, manifest, digest)


def test_fixed_torso_v2_is_rejected_even_if_wrapped_ready(tmp_path):
    manifest, digest = _fixture(
        tmp_path,
        path_schema=(
            "h1_moveit/right_handle_q0_q90_fixed_torso_natural_v2_path/v2"))
    with pytest.raises(MODULE.ExecutionError, match="PATH_SCHEMA"):
        MODULE.validate_bundle(tmp_path, manifest, digest)


def test_explicit_certified_torso_is_not_old_fixed_45_endpoint(tmp_path):
    manifest, digest = _fixture(tmp_path, torso=math.radians(39.25))
    bundle = MODULE.validate_bundle(tmp_path, manifest, digest)
    first = MODULE.build_stages(bundle, "prefix")[1].states[-1]
    assert first["torso_joint"] == pytest.approx(math.radians(39.25))
    assert first["torso_joint"] != pytest.approx(math.radians(45.0))


def test_torso_above_user_hard_limit_fails_closed(tmp_path):
    manifest, digest = _fixture(tmp_path, torso=math.radians(45.01))
    with pytest.raises(MODULE.ExecutionError, match="LIMIT_TORSO_HARD"):
        MODULE.validate_bundle(tmp_path, manifest, digest)


def test_dense_points_obey_revolute_and_prismatic_limits(tmp_path):
    manifest, digest = _fixture(tmp_path)
    bundle = MODULE.validate_bundle(tmp_path, manifest, digest)
    for stage in MODULE.build_stages(bundle, "prefix"):
        dense = MODULE.densify_stage(stage, bundle.limits)
        controlled = [name for topic in stage.topics
                      for name in MODULE.CONTROLLER_JOINTS[topic]]
        for first, second in zip(dense, dense[1:]):
            for name in controlled:
                limit = (bundle.limits.dense_prismatic_step_m
                         if "finger" in name
                         else bundle.limits.dense_angular_step_rad)
                assert abs(second[name] - first[name]) <= limit + 1e-12


def test_suffix_egress_is_exact_reverse_and_release_occurs_only_at_q0(
        tmp_path):
    manifest, digest = _fixture(tmp_path)
    bundle = MODULE.validate_bundle(tmp_path, manifest, digest)
    prefix = MODULE.build_stages(bundle, "prefix")
    suffix = MODULE.build_stages(bundle, "suffix")
    ingress = prefix[1].states
    egress = suffix[2].states
    controlled = ("torso_joint", *MODULE.RIGHT_ARM_JOINTS)
    assert [tuple(state[name] for name in controlled) for state in egress] == [
        tuple(state[name] for name in controlled)
        for state in reversed(ingress)
    ]
    release = suffix[1]
    assert release.states[0]["torso_joint"] == pytest.approx(
        bundle.rows[0]["torso_joint"])
    assert release.states[-1]["right_dex1_finger_1_joint"] == -0.019
    assert suffix[0].states[0]["right_dex1_finger_1_joint"] == pytest.approx(
        MODULE.Q90_MIDDLE_SEAM_RIGHT_GRIPPER[0])


def test_runtime_torso_watchdog_latches_violation():
    feedback = MODULE._JointFeedback()

    class Message:
        name = ["torso_joint"]
        position = [MODULE.HARD_MAX_TORSO_RAD + 1e-6]

    feedback.update(Message())
    with pytest.raises(MODULE.ExecutionError, match="RUNTIME_TORSO_LIMIT"):
        feedback.require_torso_safe()


def test_runtime_torso_watchdog_accepts_planned_peak_roundoff_with_margin():
    feedback = MODULE._JointFeedback()

    class Message:
        name = ["torso_joint"]
        position = [MODULE.PLANNED_MAX_TORSO_RAD + 1.0591527654923993e-12]

    assert (Message.position[0] - MODULE.PLANNED_MAX_TORSO_RAD
            ) == pytest.approx(1.0591527654923993e-12, abs=1e-24)
    assert Message.position[0] < MODULE.HARD_MAX_TORSO_RAD
    feedback.update(Message())
    feedback.require_torso_safe()


@pytest.mark.parametrize("sign", (-1.0, 1.0))
def test_runtime_torso_watchdog_accepts_absolute_tolerance_boundary(sign):
    feedback = MODULE._JointFeedback()

    class Message:
        name = ["torso_joint"]
        position = [sign * (MODULE.HARD_MAX_TORSO_RAD +
                            MODULE.RUNTIME_TORSO_ABS_TOL_RAD)]

    feedback.update(Message())
    feedback.require_torso_safe()


@pytest.mark.parametrize("sign", (-1.0, 1.0))
def test_runtime_torso_watchdog_rejects_beyond_absolute_tolerance(sign):
    feedback = MODULE._JointFeedback()

    class Message:
        name = ["torso_joint"]
        position = [sign * (MODULE.HARD_MAX_TORSO_RAD +
                            1.01 * MODULE.RUNTIME_TORSO_ABS_TOL_RAD)]

    feedback.update(Message())
    with pytest.raises(MODULE.ExecutionError, match="RUNTIME_TORSO_LIMIT"):
        feedback.require_torso_safe()


@pytest.mark.parametrize("value", (math.nan, math.inf, -math.inf))
def test_runtime_torso_watchdog_rejects_nonfinite_feedback(value):
    feedback = MODULE._JointFeedback()

    class Message:
        name = ["torso_joint"]
        position = [value]

    feedback.update(Message())
    with pytest.raises(MODULE.ExecutionError, match="RUNTIME_TORSO_LIMIT"):
        feedback.require_torso_safe()


def test_joint_feedback_requires_every_requested_joint_fresh():
    feedback = MODULE._JointFeedback()

    class Initial:
        name = ["torso_joint", "right_shoulder_pitch_joint"]
        position = [0.0, 0.0]

    feedback.update(Initial())
    gate = __import__("time").monotonic_ns()

    class Partial:
        name = ["torso_joint"]
        position = [0.0]

    feedback.update(Partial())
    assert feedback.snapshot(
        ["torso_joint", "right_shoulder_pitch_joint"], gate) is None

    class Rest:
        name = ["right_shoulder_pitch_joint"]
        position = [0.0]

    feedback.update(Rest())
    assert feedback.snapshot(
        ["torso_joint", "right_shoulder_pitch_joint"], gate) is not None


def _endpoint_settle_rig(
        monkeypatch, samples, *, clock_step_ns=100_000_000,
        wall_step_s=0.01, publisher_count=1, wall_deadline=10.0,
        stage_name="endpoint_test"):
    names = (
        "torso_joint", "right_shoulder_pitch_joint",
        "right_dex1_finger_1_joint",
    )
    expected = {name: 0.0 for name in names}
    limits = MODULE.Limits(
        maximum_abs_torso_rad=0.7,
        dense_angular_step_rad=MODULE.HARD_DENSE_ANGULAR_STEP_RAD,
        dense_prismatic_step_m=MODULE.HARD_DENSE_PRISMATIC_STEP_M,
        start_tolerance_rad=MODULE.HARD_START_TOLERANCE_RAD,
        endpoint_tolerance_rad=MODULE.HARD_ENDPOINT_TOLERANCE_RAD,
        gripper_tolerance_m=MODULE.HARD_GRIPPER_TOLERANCE_M,
        lead_time_s=MODULE.HARD_MIN_LEAD_TIME_S,
        point_dt_s=MODULE.HARD_MIN_POINT_DT_S,
    )
    wall = [0.0]
    receipt_ns = [100]
    clock_ns = [5_000_000_000]
    monkeypatch.setattr(MODULE.time, "monotonic", lambda: wall[0])
    monkeypatch.setattr(MODULE.time, "monotonic_ns", lambda: receipt_ns[0])
    feedback = MODULE._JointFeedback()

    def update(values):
        message = types.SimpleNamespace(
            name=list(names),
            position=[values[name] for name in names],
        )
        feedback.update(message)

    # This endpoint-looking sample predates entry into the settle helper.  The
    # helper must establish a newer lower bound after sim_target and reject it.
    update(expected)
    receipt_ns[0] += 1
    pending = list(samples)

    class Executor:
        def __init__(self):
            self.spin_calls = 0

        def spin_once(self, *, timeout_sec):
            assert timeout_sec == pytest.approx(0.02)
            self.spin_calls += 1
            wall[0] += wall_step_s
            clock_ns[0] += clock_step_ns
            if pending:
                overrides = pending.pop(0)
                if overrides is not None:
                    receipt_ns[0] += 1
                    update({**expected, **overrides})

    class Clock:
        def now(self):
            return types.SimpleNamespace(nanoseconds=clock_ns[0])

    class Node:
        def get_name(self):
            return "endpoint_settle_test"

        def get_clock(self):
            return Clock()

        def get_publishers_info_by_topic(self, _topic):
            return [types.SimpleNamespace(node_name=self.get_name())
                    for _ in range(publisher_count)]

    executor = Executor()
    node = Node()

    def run():
        MODULE._settle_stage_endpoint(
            executor, feedback, node, names, expected, limits, stage_name,
            clock_ns[0], wall_deadline)

    return types.SimpleNamespace(
        run=run, executor=executor, feedback=feedback, names=names,
        expected=expected, limits=limits)


def test_endpoint_settle_never_counts_cached_pre_target_sample(monkeypatch):
    rig = _endpoint_settle_rig(monkeypatch, [{}, {}])
    with pytest.raises(MODULE.ExecutionError) as caught:
        rig.run()
    message = str(caught.value)
    assert "STAGE_ENDPOINT:endpoint_test" in message
    assert "reason=INSUFFICIENT_CONSECUTIVE_SAMPLES" in message
    assert "fresh_samples=2" in message
    assert "consecutive=2/3" in message
    assert rig.executor.spin_calls == 5


def test_endpoint_settle_no_fresh_sample_does_not_report_cached_measurement(
        monkeypatch):
    rig = _endpoint_settle_rig(monkeypatch, [])
    with pytest.raises(MODULE.ExecutionError) as caught:
        rig.run()
    message = str(caught.value)
    fields = dict(item.split("=", 1) for item in message.split(":")[2:])
    assert fields["reason"] == "NO_POST_TARGET_FRESH_FEEDBACK"
    assert fields["fresh_samples"] == "0"
    assert math.isnan(float(fields["measured"]))


def test_endpoint_settle_allows_transient_then_three_fresh_close_samples(
        monkeypatch):
    rig = _endpoint_settle_rig(monkeypatch, [
        {"right_shoulder_pitch_joint": 0.01}, {}, {}, {},
    ])
    rig.run()
    assert rig.executor.spin_calls == 4


def test_endpoint_settle_one_fresh_close_sample_is_insufficient(monkeypatch):
    rig = _endpoint_settle_rig(monkeypatch, [{}])
    with pytest.raises(MODULE.ExecutionError) as caught:
        rig.run()
    assert "STAGE_ENDPOINT:endpoint_test" in str(caught.value)
    assert "fresh_samples=1" in str(caught.value)
    assert "consecutive=1/3" in str(caught.value)


def test_endpoint_settle_permanent_failure_reports_worst_joint(monkeypatch):
    bad = {
        "right_shoulder_pitch_joint": 0.012,
        "right_dex1_finger_1_joint": 0.002,
    }
    rig = _endpoint_settle_rig(
        monkeypatch, [bad] * 5, stage_name="permanent")
    with pytest.raises(MODULE.ExecutionError) as caught:
        rig.run()
    message = str(caught.value)
    assert message.startswith("STAGE_ENDPOINT:permanent:")
    fields = dict(item.split("=", 1) for item in message.split(":")[2:])
    assert fields["reason"] == "OUT_OF_TOLERANCE"
    assert fields["fresh_samples"] == "5"
    assert fields["worst_joint"] == "right_shoulder_pitch_joint"
    assert float(fields["measured"]) == pytest.approx(0.012)
    assert float(fields["expected"]) == pytest.approx(0.0)
    assert float(fields["error"]) == pytest.approx(0.012)
    assert float(fields["tolerance"]) == pytest.approx(0.003)


def test_endpoint_settle_keeps_torso_watchdog(monkeypatch):
    rig = _endpoint_settle_rig(monkeypatch, [{
        "torso_joint": MODULE.HARD_MAX_TORSO_RAD + 1e-6,
    }])
    with pytest.raises(MODULE.ExecutionError, match="RUNTIME_TORSO_LIMIT"):
        rig.run()


def test_endpoint_settle_keeps_unique_publisher_watchdog(monkeypatch):
    rig = _endpoint_settle_rig(monkeypatch, [], publisher_count=2)
    with pytest.raises(
            MODULE.ExecutionError,
            match="STAGE_SETTLE:endpoint_test:PUBLISHER_COUNT"):
        rig.run()


def test_endpoint_settle_keeps_sim_clock_stall_watchdog(monkeypatch):
    rig = _endpoint_settle_rig(
        monkeypatch, [], clock_step_ns=0, wall_step_s=0.4)
    with pytest.raises(
            MODULE.ExecutionError,
            match="STAGE_SIM_CLOCK_STALL:endpoint_test"):
        rig.run()


def test_endpoint_settle_keeps_wall_timeout_watchdog(monkeypatch):
    rig = _endpoint_settle_rig(
        monkeypatch, [], wall_step_s=0.1, wall_deadline=0.05)
    with pytest.raises(
            MODULE.ExecutionError, match="STAGE_SIM_TIMEOUT:endpoint_test"):
        rig.run()


@pytest.mark.parametrize(("mutation", "status"), [
    (lambda path: path.chmod(0o600), "SEALED_MODE"),
    (lambda path: None, "SEALED_SHA256"),
])
def test_manifest_file_identity_and_sha_fail_closed(tmp_path, mutation, status):
    manifest, digest = _fixture(tmp_path)
    mutation(manifest)
    expected = digest if status == "SEALED_MODE" else "0" * 64
    with pytest.raises(MODULE.ExecutionError, match=status):
        MODULE.validate_bundle(tmp_path, manifest, expected)


def test_source_has_only_canonical_robot_topics_and_zero_derivatives():
    source = SOURCE.read_text(encoding="utf-8")
    for topic in MODULE.CANONICAL_CONTROLLER_TOPICS:
        assert topic in source
    assert "/incubator/" not in source
    assert "gz.transport" not in source
    assert "set_entity_pose" not in source
    assert "point.velocities = [0.0]" in source
    assert "point.accelerations = [0.0]" in source
    assert "point.effort = []" in source
    assert source.index("validate_bundle(args.workspace") < source.rindex("_execute_ros(bundle")
    assert "feedback.latest(names)" in source
    assert "STAGE_SIM_CLOCK_STALL" in source
    assert "get_publishers_info_by_topic" in source
    assert "HARD_MAX_TIME_SCALE" in source
    assert MODULE.HARD_ENDPOINT_TOLERANCE_RAD == 0.003
    assert MODULE.HARD_GRIPPER_TOLERANCE_M == 0.001
    assert MODULE.HARD_ENDPOINT_SETTLE_SIM_S == 0.5
    assert MODULE.HARD_ENDPOINT_CONSECUTIVE_SAMPLES == 3
    assert "publication_ns" not in source


def test_controller_and_recorder_gate_requires_two_subscriptions():
    class Publisher:
        def __init__(self, count):
            self.count = count

        def get_subscription_count(self):
            return self.count

    assert MODULE.REQUIRED_CONTROLLER_AND_RECORDER_SUBSCRIPTIONS == 2
    assert not MODULE._controller_and_recorder_ready({
        "controller-only": Publisher(1), "ready": Publisher(2)})
    assert MODULE._controller_and_recorder_ready({
        "left": Publisher(2), "right": Publisher(2)})


def _launch_state(bundle):
    state = MODULE._base_state(MODULE.LAUNCH_LEFT_ARM)
    waypoint = bundle.ingress_waypoints[0]
    state["torso_joint"] = waypoint[0]
    state.update(zip(MODULE.RIGHT_ARM_JOINTS, waypoint[1:]))
    return state


def _install_fake_ros(
        monkeypatch, initial_state, *, fail_first_spin=False,
        recorder_ready_at_spin=0):
    """Install an executor-aware in-process ROS mock without a default context."""
    runtime = types.SimpleNamespace(
        state=dict(initial_state),
        clock_ns=0,
        contexts=[],
        executors=[],
        nodes=[],
        publications=[],
        subscription_samples=[],
        controller_only_publication_counts=[],
        publication_subscription_counts=[],
        default_spin_calls=0,
        fail_first_spin=fail_first_spin,
        recorder_ready_at_spin=recorder_ready_at_spin,
    )

    class Context:
        def __init__(self):
            self.initialized = False
            self.shutdown_called = False
            runtime.contexts.append(self)

    class JointState:
        name = ()
        position = ()

    class _Clock:
        ros_time_is_active = True

        def now(self):
            runtime.clock_ns += 100_000_000
            return types.SimpleNamespace(nanoseconds=runtime.clock_ns)

    class _Publisher:
        def __init__(self, node, topic):
            self.node = node
            self.topic = topic
            self.destroyed = False

        def get_subscription_count(self):
            owner = next(
                executor for executor in runtime.executors
                if executor.node is self.node)
            count = (2 if owner.spin_calls >= runtime.recorder_ready_at_spin
                     else 1)
            runtime.subscription_samples.append(
                (owner.spin_calls, self.topic, count))
            return count

        def publish(self, message):
            assert not self.destroyed
            owner = next(
                executor for executor in runtime.executors
                if executor.node is self.node)
            count = (2 if owner.spin_calls >= runtime.recorder_ready_at_spin
                     else 1)
            runtime.publication_subscription_counts.append(count)
            runtime.publications.append((owner, self.topic, message))
            point = message.points[-1]
            runtime.state.update(zip(message.joint_names, point.positions))

    class Node:
        def __init__(self, name, *, parameter_overrides,
                     automatically_declare_parameters_from_overrides, context):
            assert parameter_overrides
            assert automatically_declare_parameters_from_overrides is True
            assert context.initialized
            self.context = context
            self.name = name
            self.clock = _Clock()
            self.feedback_callback = None
            self.publishers = {}
            self.destroyed = False
            runtime.nodes.append(self)

        def get_name(self):
            return self.name

        def get_clock(self):
            return self.clock

        def create_subscription(self, _message_type, topic, callback, depth):
            assert topic == "/joint_states"
            assert depth == 10
            self.feedback_callback = callback
            return types.SimpleNamespace(callback=callback)

        def destroy_subscription(self, subscription):
            assert subscription.callback is self.feedback_callback
            self.feedback_callback = None

        def create_publisher(self, _message_type, topic, _qos):
            publisher = _Publisher(self, topic)
            self.publishers[topic] = publisher
            return publisher

        def destroy_publisher(self, publisher):
            assert self.publishers[publisher.topic] is publisher
            publisher.destroyed = True
            del self.publishers[publisher.topic]

        def get_publishers_info_by_topic(self, topic):
            if topic not in self.publishers:
                return []
            return [types.SimpleNamespace(node_name=self.name)]

        def destroy_node(self):
            self.destroyed = True

    class SingleThreadedExecutor:
        def __init__(self, *, context):
            assert context.initialized
            self.context = context
            self.node = None
            self.added_node = None
            self.removed_node = None
            self.spin_calls = 0
            self.shutdown_called = False
            runtime.executors.append(self)

        def add_node(self, node):
            assert self.node is None
            assert node.context is self.context
            self.node = node
            self.added_node = node
            return True

        def remove_node(self, node):
            assert self.node is node
            self.removed_node = node
            self.node = None

        def spin_once(self, *, timeout_sec):
            assert timeout_sec == pytest.approx(0.02)
            assert self.context.initialized
            assert self.node is not None
            self.spin_calls += 1
            if self.spin_calls < runtime.recorder_ready_at_spin:
                runtime.controller_only_publication_counts.append(
                    len(runtime.publications))
            if runtime.fail_first_spin:
                runtime.fail_first_spin = False
                raise MODULE.ExecutionError("FAKE_PRIMARY_FAILURE")
            if self.node.feedback_callback is not None:
                message = JointState()
                message.name = list(runtime.state)
                message.position = [runtime.state[name] for name in message.name]
                self.node.feedback_callback(message)

        def shutdown(self):
            assert self.node is None
            self.shutdown_called = True
            return True

    class Parameter:
        class Type:
            BOOL = "bool"

        def __init__(self, name, parameter_type, value):
            self.name = name
            self.parameter_type = parameter_type
            self.value = value

    class QoSProfile:
        def __init__(self, **kwargs):
            self.settings = kwargs

    class ReliabilityPolicy:
        RELIABLE = "reliable"

    class DurabilityPolicy:
        VOLATILE = "volatile"

    class SignalHandlerOptions:
        NO = "no"

    class _Duration:
        def __init__(self):
            self.sec = 0
            self.nanosec = 0

    class JointTrajectoryPoint:
        def __init__(self):
            self.positions = []
            self.velocities = []
            self.accelerations = []
            self.effort = []
            self.time_from_start = _Duration()

    class JointTrajectory:
        def __init__(self):
            self.joint_names = []
            self.points = []

    rclpy_module = types.ModuleType("rclpy")
    rclpy_module.__path__ = []

    def init(*, context, signal_handler_options):
        assert signal_handler_options == SignalHandlerOptions.NO
        assert not context.initialized
        context.initialized = True

    def shutdown(*, context):
        assert context.initialized
        context.initialized = False
        context.shutdown_called = True

    def forbidden_default_spin(*_args, **_kwargs):
        runtime.default_spin_calls += 1
        raise AssertionError("default/global rclpy.spin_once was used")

    rclpy_module.init = init
    rclpy_module.shutdown = shutdown
    rclpy_module.spin_once = forbidden_default_spin
    modules = {
        "rclpy": rclpy_module,
        "rclpy.context": types.ModuleType("rclpy.context"),
        "rclpy.executors": types.ModuleType("rclpy.executors"),
        "rclpy.node": types.ModuleType("rclpy.node"),
        "rclpy.parameter": types.ModuleType("rclpy.parameter"),
        "rclpy.qos": types.ModuleType("rclpy.qos"),
        "rclpy.signals": types.ModuleType("rclpy.signals"),
        "sensor_msgs": types.ModuleType("sensor_msgs"),
        "sensor_msgs.msg": types.ModuleType("sensor_msgs.msg"),
        "trajectory_msgs": types.ModuleType("trajectory_msgs"),
        "trajectory_msgs.msg": types.ModuleType("trajectory_msgs.msg"),
    }
    modules["rclpy.context"].Context = Context
    modules["rclpy.executors"].SingleThreadedExecutor = SingleThreadedExecutor
    modules["rclpy.node"].Node = Node
    modules["rclpy.parameter"].Parameter = Parameter
    modules["rclpy.qos"].QoSProfile = QoSProfile
    modules["rclpy.qos"].ReliabilityPolicy = ReliabilityPolicy
    modules["rclpy.qos"].DurabilityPolicy = DurabilityPolicy
    modules["rclpy.signals"].SignalHandlerOptions = SignalHandlerOptions
    modules["sensor_msgs"].__path__ = []
    modules["sensor_msgs"].msg = modules["sensor_msgs.msg"]
    modules["sensor_msgs.msg"].JointState = JointState
    modules["trajectory_msgs"].__path__ = []
    modules["trajectory_msgs"].msg = modules["trajectory_msgs.msg"]
    modules["trajectory_msgs.msg"].JointTrajectory = JointTrajectory
    modules["trajectory_msgs.msg"].JointTrajectoryPoint = JointTrajectoryPoint
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    return runtime


def _serialized_trajectory_publications(runtime):
    return [
        {
            "topic": topic,
            "joint_names": list(message.joint_names),
            "points": [
                {
                    "positions": list(point.positions),
                    "velocities": list(point.velocities),
                    "accelerations": list(point.accelerations),
                    "effort": list(point.effort),
                    "time_from_start": [
                        point.time_from_start.sec,
                        point.time_from_start.nanosec,
                    ],
                }
                for point in message.points
            ],
        }
        for _owner, topic, message in runtime.publications
    ]


def _trajectory_sequence_sha256(payloads):
    encoded = json.dumps(
        payloads, sort_keys=True, separators=(",", ":"),
        allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _expected_right_trajectory_publications(
        bundle, direction, time_scale):
    payloads = []
    for stage in MODULE.build_stages(bundle, direction):
        dense = MODULE.densify_stage(stage, bundle.limits)
        for topic in stage.topics:
            joint_names = MODULE.CONTROLLER_JOINTS[topic]
            points = []
            for index, state in enumerate(dense):
                seconds = (
                    bundle.limits.lead_time_s +
                    index * bundle.limits.point_dt_s * time_scale)
                sec, nanosec = MODULE._duration_message(seconds)
                points.append({
                    "positions": [state[name] for name in joint_names],
                    "velocities": [0.0] * len(joint_names),
                    "accelerations": [0.0] * len(joint_names),
                    "effort": [],
                    "time_from_start": [sec, nanosec],
                })
            payloads.append({
                "topic": topic,
                "joint_names": list(joint_names),
                "points": points,
            })
    return payloads


def test_controller_only_never_publishes_and_recorder_join_preserves_sequence(
        tmp_path, monkeypatch):
    manifest, digest = _fixture(tmp_path)
    bundle = MODULE.validate_bundle(tmp_path, manifest, digest)
    initial_state = _launch_state(bundle)
    expected = _expected_right_trajectory_publications(
        bundle, "prefix", 1.0)

    immediate = _install_fake_ros(
        monkeypatch, initial_state, recorder_ready_at_spin=0)
    MODULE._execute_ros(bundle, "prefix", 1.0, 1.0, 1.0)
    immediate_payloads = _serialized_trajectory_publications(immediate)

    delayed = _install_fake_ros(
        monkeypatch, initial_state, recorder_ready_at_spin=4)
    MODULE._execute_ros(bundle, "prefix", 1.0, 1.0, 1.0)
    delayed_payloads = _serialized_trajectory_publications(delayed)

    assert delayed.controller_only_publication_counts == [0, 0, 0]
    assert any(count == 1 for _spin, _topic, count
               in delayed.subscription_samples)
    assert any(count == 2 for _spin, _topic, count
               in delayed.subscription_samples)
    assert delayed.publication_subscription_counts
    assert set(delayed.publication_subscription_counts) == {2}

    assert delayed_payloads[0] == expected[0]
    assert immediate_payloads == expected
    assert delayed_payloads == expected
    expected_topics = [
        topic
        for stage in MODULE.build_stages(bundle, "prefix")
        for topic in stage.topics
    ]
    assert [payload["topic"] for payload in delayed_payloads] == expected_topics
    assert len(delayed_payloads) == sum(
        len(stage.topics)
        for stage in MODULE.build_stages(bundle, "prefix"))
    assert (_trajectory_sequence_sha256(delayed_payloads) ==
            _trajectory_sequence_sha256(immediate_payloads) ==
            _trajectory_sequence_sha256(expected))


def test_source_all_spin_paths_are_bound_to_explicit_custom_executor():
    source = SOURCE.read_text(encoding="utf-8")
    tree = ast.parse(source)
    spin_calls = [
        node for node in ast.walk(tree)
        if (isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "spin_once")
    ]
    assert len(spin_calls) == 8
    assert all(
        isinstance(call.func.value, ast.Name)
        and call.func.value.id == "executor"
        for call in spin_calls)
    assert "rclpy.spin_once" not in source
    assert source.count("SingleThreadedExecutor(context=context)") == 3
    assert source.count("executor.add_node(node)") == 3
    assert source.count("executor.remove_node(node)") == 3
    assert source.count("executor.shutdown()") == 3
    wait_function = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "_wait_feedback")
    assert wait_function.args.args[0].arg == "executor"
    execute_function = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "_execute_ros")
    execute_source = ast.get_source_segment(source, execute_function)
    hold_function = next(
        node for node in ast.walk(execute_function)
        if isinstance(node, ast.FunctionDef) and node.name == "hold_current")
    hold_source = ast.get_source_segment(source, hold_function)
    assert "_wait_feedback(\n                executor" in hold_source
    assert "executor.spin_once(timeout_sec=0.02)" in hold_source
    assert "SingleThreadedExecutor(context=context)" in execute_source


def test_prefix_checker_suffix_checker_reinitialize_custom_contexts_in_process(
        tmp_path, monkeypatch):
    manifest, digest = _fixture(tmp_path)
    bundle = MODULE.validate_bundle(tmp_path, manifest, digest)
    runtime = _install_fake_ros(monkeypatch, _launch_state(bundle))
    monkeypatch.setattr(MODULE, "validate_bundle", lambda *_args: bundle)
    monkeypatch.setattr(
        MODULE, "densify_stage", lambda stage, _limits: stage.states)

    MODULE._execute_ros(bundle, "prefix", 1.0, 0.1, 1.0)
    MODULE.check_q90_seam(tmp_path, manifest, digest, 0.1)
    MODULE._execute_ros(bundle, "suffix", 1.0, 0.1, 1.0)
    MODULE.check_launch_seam(tmp_path, manifest, digest, 0.1)

    assert runtime.default_spin_calls == 0
    assert len(runtime.contexts) == 4
    assert len({id(context) for context in runtime.contexts}) == 4
    assert len(runtime.executors) == 4
    assert len(runtime.nodes) == 4
    for context, executor, node in zip(
            runtime.contexts, runtime.executors, runtime.nodes):
        assert executor.context is context
        assert executor.added_node is node
        assert executor.removed_node is node
        assert executor.node is None
        assert executor.spin_calls > 0
        assert executor.shutdown_called
        assert node.context is context
        assert node.destroyed
        assert context.shutdown_called
        assert not context.initialized
    expected = _launch_state(bundle)
    assert runtime.state == pytest.approx(expected)


def test_failure_hold_spins_on_same_custom_executor(tmp_path, monkeypatch,
                                                    capsys):
    manifest, digest = _fixture(tmp_path)
    bundle = MODULE.validate_bundle(tmp_path, manifest, digest)
    runtime = _install_fake_ros(
        monkeypatch, _launch_state(bundle), fail_first_spin=True)
    tick = [0.0]

    def monotonic():
        tick[0] += 0.05
        return tick[0]

    monkeypatch.setattr(MODULE.time, "monotonic", monotonic)
    with pytest.raises(MODULE.ExecutionError, match="FAKE_PRIMARY_FAILURE"):
        MODULE._execute_ros(bundle, "prefix", 1.0, 0.1, 1.0)

    output = capsys.readouterr()
    assert "RIGHT_HANDLE_EXECUTOR_828_FAILURE_HOLD=PASS" in output.out
    assert output.err == ""
    assert runtime.default_spin_calls == 0
    assert len(runtime.executors) == 1
    executor = runtime.executors[0]
    assert executor.spin_calls >= 2
    assert len(runtime.publications) == len(MODULE.CANONICAL_CONTROLLER_TOPICS)
    assert all(owner is executor for owner, _topic, _message
               in runtime.publications)
    assert executor.shutdown_called
    assert executor.removed_node is executor.added_node
    assert executor.added_node.destroyed
    assert runtime.contexts[0].shutdown_called
