"""Offline unit tests for the deterministic 829 torso-margin generator."""

import copy
import hashlib
import importlib.util
import math
import stat
import sys
from pathlib import Path

import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
SOURCE = WORKSPACE / "src/h1_2_moveit_demo/scripts/h1_right_handle_torso_margin_829.py"
V3 = WORKSPACE / (
    "runtime_assets/right_handle_8263_natural/variable_torso_v3_828/"
    "contact_path_v3.json")
URDF = WORKSPACE / (
    "runtime_evidence/826_apriltag_plan_only_20260826/stage_h_nominal_pass/"
    "clearance_audit_robot.urdf")
SPEC = importlib.util.spec_from_file_location("h1_right_handle_torso_margin_829_test", SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class _IdentityRetargetModel:
    arm_limits = tuple((-10.0, 10.0) for _ in MODULE.RIGHT_ARM_JOINTS)
    solver_identity = {
        "implementation": "unit-test-identity",
        "module_sha256": "0" * 64,
        "python_implementation": "test",
        "python_version": "0",
        "version": "0",
    }

    def validate_source_limits(self, rows):
        assert rows

    def retarget(self, source_torso_rad, source_arm_rad, target_torso_rad,
                 previous_values, next_values):
        assert source_torso_rad > target_torso_rad
        assert len(previous_values) == 8
        assert len(next_values) == 8
        return MODULE.SolveResult(
            tuple(source_arm_rad), 2.0e-9, 3.0e-9, "identity_test")


@pytest.fixture(scope="module")
def formal_source():
    document, digest = MODULE.load_source(V3, MODULE.DEFAULT_SOURCE_SHA256)
    MODULE.validate_source_document(document)
    return document, digest


@pytest.fixture(scope="module")
def formal_model():
    model, digest = MODULE.load_model(URDF, MODULE.DEFAULT_URDF_SHA256)
    assert digest == MODULE.DEFAULT_URDF_SHA256
    return model


@pytest.fixture(scope="module")
def formal_candidate(formal_source, formal_model):
    document, digest = formal_source
    generator_sha = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    return MODULE.generate_candidate(
        document, formal_model, digest, MODULE.DEFAULT_URDF_SHA256,
        generator_sha, "2026-08-29T00:00:00+08:00")


def test_formal_sha_shape_and_single_hotspot_pykdl_retarget(
        formal_source, formal_model):
    document, _ = formal_source
    model = formal_model
    assert model.full.joint_names == (MODULE.TORSO_JOINT, *MODULE.RIGHT_ARM_JOINTS)
    assert model.arm.joint_names == MODULE.RIGHT_ARM_JOINTS
    row = document["rows"][50]
    previous = document["rows"][49]
    following = document["rows"][51]
    result = model.retarget(
        math.radians(row["torso_yaw_deg"]), row["right_arm_rad"],
        math.radians(MODULE.PLANNED_TORSO_LIMIT_DEG),
        (math.radians(previous["torso_yaw_deg"]), *previous["right_arm_rad"]),
        (math.radians(MODULE.PLANNED_TORSO_LIMIT_DEG), *following["right_arm_rad"]),
    )
    assert len(result.arm_rad) == 7
    assert all(math.isfinite(value) for value in result.arm_rad)
    assert result.position_error_m <= MODULE.POSITION_TOLERANCE_M
    assert result.orientation_error_rad <= MODULE.ORIENTATION_TOLERANCE_RAD


def test_candidate_preserves_geometry_endpoints_and_is_fail_closed(
        formal_source, formal_candidate):
    document, digest = formal_source
    assert digest == MODULE.DEFAULT_SOURCE_SHA256
    candidate, summary = formal_candidate
    assert candidate["$schema"] == MODULE.OUTPUT_SCHEMA
    assert candidate["schema_version"] == 4
    assert candidate["contract"]["torso_abs_limit_deg"] == 44.5
    assert candidate["status"]["execution_ready"] is False
    assert candidate["status"]["full_execution_topology_exact_complete"] is False
    assert candidate["status"]["ik_retarget_complete"] is True
    assert candidate["endpoints"] == document["endpoints"]
    assert candidate["rows"][0] == document["rows"][0]
    assert candidate["rows"][-1] == document["rows"][-1]
    expected_retargeted = 0
    for before, after in zip(document["rows"], candidate["rows"]):
        for key in MODULE.GEOMETRY_KEYS:
            assert after[key] == before[key]
        if before["torso_yaw_deg"] > MODULE.PLANNED_TORSO_LIMIT_DEG:
            expected_retargeted += 1
            assert after["torso_yaw_deg"] == MODULE.PLANNED_TORSO_LIMIT_DEG
        else:
            assert after == before
    assert summary["result"]["retargeted_row_count"] == expected_retargeted
    assert expected_retargeted == 507
    assert summary["result"]["first_retargeted_row"] == 45
    assert summary["result"]["last_retargeted_row"] == 551
    assert summary["result"]["minimum_arm_margin_deg"] >= 2.0
    position_error_mm = summary["result"]["maximum_tcp_position_retarget_error_mm"]
    orientation_error_deg = summary["result"][
        "maximum_tcp_orientation_retarget_error_deg"]
    assert position_error_mm <= MODULE.POSITION_TOLERANCE_M * 1000.0
    assert orientation_error_deg <= math.degrees(MODULE.ORIENTATION_TOLERANCE_RAD)
    assert any(
        after["right_arm_rad"] != before["right_arm_rad"]
        for before, after in zip(document["rows"], candidate["rows"])
        if before["torso_yaw_deg"] > MODULE.PLANNED_TORSO_LIMIT_DEG)
    assert max(abs(row["torso_yaw_deg"]) for row in candidate["rows"]) <= 44.5
    tsv = MODULE.render_batch_tsv(candidate["rows"]).decode("ascii").splitlines()
    assert tsv[0].startswith("# id handle_deg torso_deg")
    assert len(tsv) == len(candidate["rows"]) + 1
    assert all(len(line.split()) == 12 for line in tsv[1:])


def test_strict_validation_rejects_nonfinite_shape_endpoint_and_continuity(formal_source):
    document, _ = formal_source
    bad = copy.deepcopy(document)
    bad["rows"][10]["z_mm"] = math.nan
    with pytest.raises(MODULE.MarginGenerationError, match="non-finite"):
        MODULE.validate_source_document(bad)

    bad = copy.deepcopy(document)
    bad["rows"].pop()
    with pytest.raises(MODULE.MarginGenerationError, match="SOURCE_ROW_COUNT"):
        MODULE.validate_source_document(bad)

    bad = copy.deepcopy(document)
    bad["endpoints"]["q0"]["z_mm"] += 0.1
    with pytest.raises(MODULE.MarginGenerationError, match="SOURCE_Q0_ENDPOINT_MISMATCH"):
        MODULE.validate_source_document(bad)

    bad = copy.deepcopy(document)
    bad["rows"][100]["right_arm_rad"][0] += math.radians(5.0)
    with pytest.raises(MODULE.MarginGenerationError, match="SOURCE_CONTINUITY"):
        MODULE.validate_source_document(bad)


def test_sha_duplicate_key_and_symlink_inputs_fail_closed(tmp_path):
    regular = tmp_path / "input.json"
    regular.write_text('{"a":1,"a":2}\n', encoding="ascii")
    digest = hashlib.sha256(regular.read_bytes()).hexdigest()
    raw, actual = MODULE._read_bound_regular_file(
        regular, digest, 1024, "TEST")
    assert actual == digest
    with pytest.raises(MODULE.MarginGenerationError, match="JSON_DUPLICATE_KEY"):
        MODULE._parse_json(raw, "TEST")
    with pytest.raises(MODULE.MarginGenerationError, match="SHA256 mismatch"):
        MODULE._read_bound_regular_file(regular, "0" * 64, 1024, "TEST")
    link = tmp_path / "link.json"
    link.symlink_to(regular)
    with pytest.raises(MODULE.MarginGenerationError, match="cannot open protected input"):
        MODULE._read_bound_regular_file(link, digest, 1024, "TEST")


def test_atomic_no_replace_publication_and_read_only_mode(tmp_path):
    json_path = tmp_path / "candidate.json"
    tsv_path = tmp_path / "candidate.tsv"
    summary_path = tmp_path / "summary.json"
    summary_path.write_bytes(b"sentinel")
    with pytest.raises(MODULE.MarginGenerationError, match="OUTPUT_EXISTS"):
        MODULE.publish_outputs_no_replace({
            json_path: b"json\n", tsv_path: b"tsv\n", summary_path: b"summary\n"})
    assert not json_path.exists()
    assert not tsv_path.exists()
    assert summary_path.read_bytes() == b"sentinel"

    summary_path.unlink()
    payloads = {
        json_path: b"json\n", tsv_path: b"tsv\n", summary_path: b"summary\n"}
    MODULE.publish_outputs_no_replace(payloads)
    for path, expected in payloads.items():
        assert path.read_bytes() == expected
        assert stat.S_IMODE(path.stat().st_mode) == 0o400
    with pytest.raises(MODULE.MarginGenerationError, match="OUTPUT_EXISTS"):
        MODULE.publish_outputs_no_replace(payloads)
    assert json_path.read_bytes() == b"json\n"


def test_explicit_timestamp_and_canonical_bytes_are_reproducible(formal_source):
    document, digest = formal_source
    generator_sha = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    arguments = (
        document, _IdentityRetargetModel(), digest, MODULE.DEFAULT_URDF_SHA256,
        generator_sha, "2026-08-29T00:00:00+08:00")
    first, first_summary = MODULE.generate_candidate(*arguments)
    second, second_summary = MODULE.generate_candidate(*arguments)
    assert MODULE._canonical_json_bytes(first) == MODULE._canonical_json_bytes(second)
    assert MODULE._canonical_json_bytes(first_summary) == MODULE._canonical_json_bytes(
        second_summary)
    with pytest.raises(MODULE.MarginGenerationError, match="CREATED_AT_RFC3339"):
        MODULE.generate_candidate(
            document, _IdentityRetargetModel(), digest, MODULE.DEFAULT_URDF_SHA256,
            generator_sha, "now")
