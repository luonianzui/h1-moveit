#!/usr/bin/python3
"""Offline unit tests for sealed trajectory replay retiming."""

from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import importlib.util
from io import StringIO
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock


TOOL_PATH = Path(__file__).parents[1] / "scripts" / "h1_joint_trajectory_asset.py"
CUBE_SUPPORT_PATH = Path(__file__).parents[1] / "scripts" / "h1_cube_support_8251.sh"
DOOR_SUPPORT_PATH = Path(__file__).parents[1] / "scripts" / "h1_door_support_8251.sh"
FULL_WRAPPER_PATH = Path(__file__).parents[1] / "scripts" / "h1_full_incubator_8251.sh"
CLOSE_DETACH_PATH = (
    Path(__file__).parents[1] / "scripts" / "h1_door_support_detach_8253.sh"
)
CLOSE_WRAPPER_PATH = (
    Path(__file__).parents[1] / "scripts" / "h1_close_and_relock_8253.sh"
)
SPEC = importlib.util.spec_from_file_location("h1_joint_trajectory_asset", TOOL_PATH)
assert SPEC is not None and SPEC.loader is not None
ASSET_TOOL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ASSET_TOOL)


def duration(seconds: float) -> dict[str, int]:
    value_ns = round(seconds * 1_000_000_000)
    return {"sec": value_ns // 1_000_000_000, "nanosec": value_ns % 1_000_000_000}


def gripper_record(
    *,
    sequence: int = 0,
    topic: str = "/left_gripper_controller/joint_trajectory",
    wall_offset_ns: int = 0,
    final_time: float = 1.0,
    final_position: float = 0.01,
    velocity: float = 0.02,
    acceleration: float = 0.04,
) -> dict[str, object]:
    return {
        "sequence": sequence,
        "topic": topic,
        "wall_offset_ns": wall_offset_ns,
        "joint_names": list(ASSET_TOOL.CONTROLLER_JOINTS[topic]),
        "points": [
            {
                "positions": [0.0, 0.0],
                "velocities": [0.0, 0.0],
                "accelerations": [0.0, 0.0],
                "effort": [1.0, 2.0],
                "time_from_start": duration(0.0),
            },
            {
                "positions": [final_position, final_position],
                "velocities": [velocity, velocity],
                "accelerations": [acceleration, acceleration],
                "effort": [3.0, 4.0],
                "time_from_start": duration(final_time),
            },
        ],
    }


def write_sealed_asset(path: Path, stage: str, messages: list[dict[str, object]]) -> str:
    payload = {
        "schema": ASSET_TOOL.SCHEMA,
        "stage": stage,
        "controller_topics": list(ASSET_TOOL.CONTROLLER_TOPICS),
        "clock": {
            "wall_offset_unit": "nanoseconds",
            "time_from_start_unit": "sec_nanosec",
        },
        "messages": messages,
    }
    ASSET_TOOL._validate_payload(payload)
    digest = ASSET_TOOL.hashlib.sha256(ASSET_TOOL._canonical_bytes(payload)).hexdigest()
    ASSET_TOOL._exclusive_write(path, {"payload": payload, "sha256": digest})
    return digest


def cube_contact_messages() -> list[dict[str, object]]:
    close = gripper_record(final_position=0.003)
    close["points"][0]["positions"] = [-0.019, -0.019]
    close["expected_actual_endpoint"] = {
        "kind": ASSET_TOOL.CUBE_CLOSE_CONTACT_ENDPOINT_KIND,
        "positions": [-0.015, -0.015],
        "desired_error_limit_rad": 0.020,
    }
    opened = gripper_record(
        sequence=1,
        wall_offset_ns=2_000_000_000,
        final_position=-0.019,
    )
    opened["points"][0]["positions"] = [-0.015, -0.015]
    return [close, opened]


def controller_sample(
    timestamp_ns: int,
    desired: list[float],
    actual: list[float],
) -> dict[str, object]:
    return {
        "timestamp_ns": timestamp_ns,
        "desired_positions": desired,
        "desired_velocities": [0.0 for _ in desired],
        "desired_accelerations": [0.0 for _ in desired],
        "desired_effort": [],
        "actual_positions": actual,
    }


def cube_contact_controller_samples() -> list[dict[str, object]]:
    return [
        controller_sample(100_000_000, [-0.019, -0.019], [-0.019, -0.019]),
        controller_sample(200_000_000, [-0.010, -0.010], [-0.017, -0.017]),
        controller_sample(300_000_000, [0.003, 0.003], [-0.015, -0.015]),
        controller_sample(360_000_000, [0.003, 0.003], [-0.015, -0.015]),
        controller_sample(600_000_000, [-0.010, -0.010], [-0.017, -0.017]),
        controller_sample(700_000_000, [-0.019, -0.019], [-0.019, -0.019]),
        controller_sample(760_000_000, [-0.019, -0.019], [-0.019, -0.019]),
    ]


def stationary_controller_samples(topic: str) -> list[dict[str, object]]:
    positions = [0.0 for _ in ASSET_TOOL.CONTROLLER_JOINTS[topic]]
    return [
        controller_sample(timestamp, list(positions), list(positions))
        for timestamp in (100_000_000, 200_000_000, 300_000_000)
    ]


def capture_state_args(contact_error: float | None) -> SimpleNamespace:
    return SimpleNamespace(
        stage="cube",
        time_scale=1.0,
        motion_threshold=0.0005,
        sample_motion_epsilon=0.000001,
        segment_gap=0.12,
        stable_window=0.05,
        stable_tolerance=0.0005,
        actual_tolerance=0.003,
        downsample_tolerance=0.0002,
        max_point_period=0.05,
        cube_close_contact_max_error=contact_error,
    )


class MessageTimeScaleParsingTest(unittest.TestCase):
    def test_accepts_canonical_unique_list(self) -> None:
        self.assertEqual(
            ASSET_TOOL._parse_message_time_scales("0:0.25,3:1,12:0.500"),
            {0: 0.25, 3: 1.0, 12: 0.5},
        )

    def test_rejects_noncanonical_or_unsafe_items(self) -> None:
        invalid = (
            "",
            "0:0",
            "0:1.0001",
            "0:-0.5",
            "0:nan",
            "0:5e-1",
            "00:0.5",
            "0:.5",
            " 0:0.5",
            "0:0.5 ",
            "0:0.5,",
            "0:0.5,0:0.6",
            "0:0.5/1:0.5",
        )
        for value in invalid:
            with self.subTest(value=value), self.assertRaises(ASSET_TOOL.AssetError):
                ASSET_TOOL._parse_message_time_scales(value)

    def test_rejects_unused_or_out_of_asset_indices(self) -> None:
        with self.assertRaisesRegex(ASSET_TOOL.AssetError, "outside selected replay range"):
            ASSET_TOOL._selected_message_time_scales({0: 0.5}, 1, 3, 4)
        with self.assertRaisesRegex(ASSET_TOOL.AssetError, "outside asset range"):
            ASSET_TOOL._selected_message_time_scales({4: 0.5}, 0, 4, 4)

    def test_default_cli_contract_is_unchanged(self) -> None:
        args = ASSET_TOOL.build_parser().parse_args(["replay", "asset.json"])
        self.assertEqual(args.time_scale, 1.0)
        self.assertIsNone(args.message_time_scales)
        self.assertEqual(args.minimum_subscriptions, 1)

    def test_minimum_subscriptions_must_be_positive(self) -> None:
        errors = StringIO()
        with redirect_stderr(errors):
            status = ASSET_TOOL.main(
                ["replay", "unused.json", "--minimum-subscriptions", "0"]
            )
        self.assertEqual(status, 2)
        self.assertIn("must be at least one", errors.getvalue())

    def test_uniform_and_per_message_options_are_mutually_exclusive(self) -> None:
        with redirect_stderr(StringIO()), self.assertRaises(SystemExit):
            ASSET_TOOL.build_parser().parse_args(
                [
                    "replay",
                    "asset.json",
                    "--time-scale",
                    "0.5",
                    "--message-time-scales",
                    "0:0.5",
                ]
            )

    def test_rejects_repeated_per_message_option(self) -> None:
        errors = StringIO()
        with redirect_stderr(errors):
            status = ASSET_TOOL.main(
                [
                    "replay",
                    "unused.json",
                    "--message-time-scales",
                    "0:0.5",
                    "--message-time-scales",
                    "1:0.5",
                ]
            )
        self.assertEqual(status, 2)
        self.assertIn("may be supplied exactly once", errors.getvalue())


class CubeContactEndpointContractTest(unittest.TestCase):
    def test_default_capture_state_cli_is_unchanged_and_scoped_option_is_explicit(self) -> None:
        args = ASSET_TOOL.build_parser().parse_args(
            ["capture-state", "asset.json", "--stage", "cube"]
        )
        self.assertIsNone(args.cube_close_contact_max_error)

    def test_scoped_option_rejects_other_stage_hard_limit_and_global_relaxation(self) -> None:
        cases = (
            ["--stage", "door", "--cube-close-contact-max-error", "0.020"],
            ["--stage", "cube", "--cube-close-contact-max-error", "0.0201"],
            [
                "--stage",
                "cube",
                "--cube-close-contact-max-error",
                "0.020",
                "--actual-tolerance",
                "0.004",
            ],
        )
        for suffix in cases:
            errors = StringIO()
            with self.subTest(suffix=suffix), redirect_stderr(errors):
                status = ASSET_TOOL.main(["capture-state", "unused.json", *suffix])
            self.assertEqual(status, 2)
            self.assertIn("h1_joint_trajectory_asset: ERROR:", errors.getvalue())

    def test_old_v1_message_shape_and_feedback_endpoint_remain_unchanged(self) -> None:
        message = gripper_record()
        payload = {
            "schema": ASSET_TOOL.SCHEMA,
            "stage": "handle",
            "controller_topics": list(ASSET_TOOL.CONTROLLER_TOPICS),
            "clock": {
                "wall_offset_unit": "nanoseconds",
                "time_from_start_unit": "sec_nanosec",
            },
            "messages": [message],
        }
        before = ASSET_TOOL._canonical_bytes(payload)
        ASSET_TOOL._validate_payload(payload)
        self.assertEqual(ASSET_TOOL._canonical_bytes(payload), before)
        self.assertIs(
            ASSET_TOOL._expected_feedback_endpoint(message),
            message["points"][-1]["positions"],
        )

    def test_cube_close_extension_is_hash_bound_and_selects_physical_endpoint(self) -> None:
        messages = cube_contact_messages()
        with tempfile.TemporaryDirectory() as directory:
            asset = Path(directory) / "cube.json"
            digest = write_sealed_asset(asset, "cube", messages)
            payload, _durations, loaded_digest = ASSET_TOOL.load_asset(asset)
        self.assertEqual(loaded_digest, digest)
        close = payload["messages"][0]
        self.assertEqual(close["points"][-1]["positions"], [0.003, 0.003])
        outgoing = ASSET_TOOL._retimed_message_points(
            close, 1.0, False, "cube close"
        )
        self.assertEqual(outgoing[-1]["positions"], [0.003, 0.003])
        self.assertEqual(
            ASSET_TOOL._expected_feedback_endpoint(close), [-0.015, -0.015]
        )
        actual_contact = {
            "left_dex1_finger_1_joint": -0.015,
            "left_dex1_finger_2_joint": -0.015,
        }
        ASSET_TOOL._require_record_feedback_endpoint(
            actual_contact, close, 0.003, "cube close"
        )
        desired_only = {
            "left_dex1_finger_1_joint": 0.003,
            "left_dex1_finger_2_joint": 0.003,
        }
        with self.assertRaisesRegex(ASSET_TOOL.AssetError, "exceeds"):
            ASSET_TOOL._require_record_feedback_endpoint(
                desired_only, close, 0.003, "cube close"
            )

    def test_extension_rejects_other_stage_oversized_error_and_missing_open_seam(self) -> None:
        messages = cube_contact_messages()
        with self.assertRaisesRegex(ASSET_TOOL.AssetError, "allowed only"):
            ASSET_TOOL._validate_payload(
                {
                    "schema": ASSET_TOOL.SCHEMA,
                    "stage": "handle",
                    "controller_topics": list(ASSET_TOOL.CONTROLLER_TOPICS),
                    "clock": {
                        "wall_offset_unit": "nanoseconds",
                        "time_from_start_unit": "sec_nanosec",
                    },
                    "messages": messages,
                }
            )

        oversized = cube_contact_messages()
        oversized[0]["expected_actual_endpoint"]["desired_error_limit_rad"] = 0.021
        with self.assertRaisesRegex(ASSET_TOOL.AssetError, "must be in"):
            ASSET_TOOL._validate_payload(
                {
                    "schema": ASSET_TOOL.SCHEMA,
                    "stage": "cube",
                    "controller_topics": list(ASSET_TOOL.CONTROLLER_TOPICS),
                    "clock": {
                        "wall_offset_unit": "nanoseconds",
                        "time_from_start_unit": "sec_nanosec",
                    },
                    "messages": oversized,
                }
            )

        no_seam = cube_contact_messages()
        no_seam[1]["points"][0]["positions"] = [-0.019, -0.019]
        with self.assertRaisesRegex(ASSET_TOOL.AssetError, "does not start"):
            ASSET_TOOL._validate_payload(
                {
                    "schema": ASSET_TOOL.SCHEMA,
                    "stage": "cube",
                    "controller_topics": list(ASSET_TOOL.CONTROLLER_TOPICS),
                    "clock": {
                        "wall_offset_unit": "nanoseconds",
                        "time_from_start_unit": "sec_nanosec",
                    },
                    "messages": no_seam,
                }
            )

    def test_default_reconstruction_rejects_contact_error(self) -> None:
        with self.assertRaisesRegex(ASSET_TOOL.AssetError, "stable actual endpoint"):
            ASSET_TOOL._reconstruct_controller_segments(
                "/left_gripper_controller/joint_trajectory",
                cube_contact_controller_samples(),
                capture_state_args(None),
            )

    def test_explicit_reconstruction_keeps_desired_and_seals_actual_endpoint(self) -> None:
        records = ASSET_TOOL._reconstruct_controller_segments(
            "/left_gripper_controller/joint_trajectory",
            cube_contact_controller_samples(),
            capture_state_args(0.020),
        )
        self.assertEqual(len(records), 2)
        close = records[0][1]
        opened = records[1][1]
        self.assertEqual(close["points"][-1]["positions"], [0.003, 0.003])
        self.assertEqual(
            close["expected_actual_endpoint"]["positions"], [-0.015, -0.015]
        )
        self.assertEqual(opened["points"][0]["positions"], [-0.015, -0.015])
        samples = {
            topic: stationary_controller_samples(topic)
            for topic in ASSET_TOOL.CONTROLLER_TOPICS
        }
        samples["/left_gripper_controller/joint_trajectory"] = (
            cube_contact_controller_samples()
        )
        payload = ASSET_TOOL._state_samples_to_payload(
            samples, capture_state_args(0.020)
        )
        self.assertEqual(len(payload["messages"]), 2)
        self.assertIn("expected_actual_endpoint", payload["messages"][0])

    def test_cube_helpers_require_the_sealed_physical_endpoint(self) -> None:
        cube_support = CUBE_SUPPORT_PATH.read_text(encoding="utf-8")
        full_wrapper = FULL_WRAPPER_PATH.read_text(encoding="utf-8")
        for source in (cube_support, full_wrapper):
            self.assertIn(
                ".expected_actual_endpoint.kind ==", source
            )
            self.assertIn("cube_gripper_contact_close", source)
        self.assertIn(
            "$message.expected_actual_endpoint.positions", cube_support
        )

    def test_cube_support_preload_runtime_ack_and_contact_contract(self) -> None:
        cube_support = CUBE_SUPPORT_PATH.read_text(encoding="utf-8")
        full_wrapper = FULL_WRAPPER_PATH.read_text(encoding="utf-8")
        preload_branch = cube_support.split(
            'if [[ "${operation}" == "preload" ]]; then', 1
        )[1].split('\nelif [[ "${operation}" == "attach" ||', 1)[0]
        attach_branch = cube_support.split(
            'elif [[ "${operation}" == "attach" ||', 1
        )[1].split("\nelse\n", 1)[0]
        detach_branch = cube_support.split(
            'elif [[ "${operation}" == "attach" ||', 1
        )[1].split("\nelse\n", 1)[1].split("\nfi", 1)[0]
        snapshot_reader = cube_support.split("read_robot_joint_rows()", 1)[1].split(
            "\n}\n\nasset_left_arm_endpoint_gate", 1
        )[0]
        attach_dynamic = cube_support.split("run_attach_dynamic_preflight()", 1)[
            1
        ].split("\n}\n\nrun_detach_dynamic_preflight", 1)[0]
        detach_dynamic = cube_support.split("run_detach_dynamic_preflight()", 1)[
            1
        ].split('\n}\n\nif [[ "${operation}" == "preload"', 1)[0]

        self.assertIn('"${operation}" == "preload"', cube_support)
        self.assertIn("run_preload_static_preflight", preload_branch)
        self.assertIn("require_empty_plugin_transport_topics", cube_support)
        self.assertIn("require_add_service_contract", cube_support)
        self.assertIn(
            "<initially_attached>false</initially_attached>", preload_branch
        )
        self.assertIn("require_state detached", preload_branch)
        self.assertIn("require_single_resident_plugin", preload_branch)
        self.assertIn("create_preload_evidence", preload_branch)
        self.assertIn("CUBE_SUPPORT_PRELOAD=PASS", preload_branch)

        self.assertIn("require_preload_evidence", attach_branch)
        self.assertIn("start_state_listener", attach_branch)
        self.assertIn("wait_for_listener_discovery after_add", attach_branch)
        self.assertLess(
            attach_branch.index("start_state_listener"),
            attach_branch.index("wait_for_listener_discovery after_add"),
        )
        self.assertLess(
            attach_branch.index("wait_for_listener_discovery after_add"),
            attach_branch.index("run_attach_dynamic_preflight"),
        )
        self.assertIn("run_attach_dynamic_preflight", attach_branch)
        self.assertIn("cube_pose_gate 0.334 0.377 0.830", attach_dynamic)
        cube_pose_reader = cube_support.split("read_cube_json()", 1)[1].split(
            "cube_json=", 1
        )[0]
        self.assertIn("for attempt in 1 2 3", cube_pose_reader)
        self.assertIn("-n 1", cube_pose_reader)
        self.assertNotIn("cube_stability_gate", attach_dynamic)
        self.assertIn("require_resident_plugin_with_listener", attach_dynamic)
        self.assertIn("asset_left_arm_endpoint_gate", attach_branch)
        self.assertIn("asset_left_finger_endpoint_gate", attach_branch)
        self.assertIn("left_contact_gate", attach_branch)
        self.assertIn('gz topic -t "${attach_topic}"', attach_branch)
        self.assertIn("require_state attached", attach_branch)
        self.assertIn("create_ownership_evidence", attach_branch)
        self.assertNotIn("gz service -s", attach_branch)
        self.assertIn('robot_joint_rows="$(<', attach_branch)
        self.assertIn("wait_for_observed_state detached", attach_branch)
        self.assertIn(
            "CUBE_SUPPORT_FAST_RESIDENT_BASELINE=PASS operation=attach state=detached",
            attach_branch,
        )

        self.assertIn("require_preload_evidence", detach_branch)
        self.assertIn("require_ownership_evidence", detach_branch)
        self.assertIn("start_state_listener", detach_branch)
        self.assertIn("wait_for_listener_discovery after_add", detach_branch)
        self.assertLess(
            detach_branch.index("start_state_listener"),
            detach_branch.index("wait_for_listener_discovery after_add"),
        )
        self.assertLess(
            detach_branch.index("wait_for_listener_discovery after_add"),
            detach_branch.index("run_detach_dynamic_preflight"),
        )
        self.assertIn("require_resident_plugin_with_listener", detach_dynamic)
        self.assertIn("cube_pose_gate 0.610890542898", detach_dynamic)
        self.assertIn('"${rack_position_tolerance}"', detach_dynamic)
        self.assertIn('"${rack_orientation_tolerance}"', detach_dynamic)
        self.assertIn(
            'H1_CUBE_RACK_POSITION_TOLERANCE:-0.003', cube_support
        )
        self.assertIn(
            'H1_CUBE_RACK_ORIENTATION_TOLERANCE:-0.034906585039887',
            cube_support,
        )
        self.assertIn('H1_CUBE_ARM_ENDPOINT_TOLERANCE:-0.003', cube_support)
        self.assertIn('H1_CUBE_RACK_EXPECTED_YAW:-70', cube_support)
        self.assertIn('H1_CUBE_SUPPORT_FAST_RESIDENT:-0', cube_support)
        self.assertIn(
            'H1_CUBE_RACK_EXPECTED_YAW must be exactly 70 or 90 degrees',
            cube_support,
        )
        self.assertIn("rack_expected_yaw_rad='1.221730476396031'", cube_support)
        self.assertIn("rack_expected_yaw_rad='1.570796326794897'", cube_support)
        self.assertIn(
            '90-degree rack target requires the two-degree orientation gate',
            cube_support,
        )
        self.assertIn("tolerance <= 0.012", cube_support)
        self.assertIn("position <= 0.030", cube_support)
        self.assertIn("orientation <= 0.523598775598299", cube_support)
        self.assertIn("cube_stability_gate", detach_dynamic)
        self.assertIn('[[ "${fast_resident}" == "0" ]]', attach_dynamic)
        self.assertIn('[[ "${fast_resident}" == "0" ]]', detach_dynamic)
        self.assertIn('"${rack_expected_yaw_rad}"', detach_dynamic)
        self.assertIn("asset_left_arm_endpoint_gate", detach_branch)
        self.assertIn("asset_left_finger_endpoint_gate", detach_branch)
        self.assertIn("left_contact_gate", detach_branch)
        self.assertIn('gz topic -t "${detach_topic}"', detach_branch)
        self.assertIn("require_state detached", detach_branch)
        self.assertIn('"${ownership_file}" "${preload_file}"', detach_branch)
        self.assertIn("wait_for_observed_state attached", detach_branch)
        self.assertIn(
            "CUBE_SUPPORT_FAST_RESIDENT_BASELINE=PASS operation=detach state=attached",
            detach_branch,
        )

        fast_observer = cube_support.split("wait_for_observed_state()", 1)[1].split(
            "\n}\n\nrun_preload_static_preflight", 1
        )[0]
        self.assertIn('"${temporary_dir}/state.txt"', fast_observer)
        self.assertIn('kill -0 "${listener_pid}"', fast_observer)
        self.assertNotIn("kill -TERM", fast_observer)

        self.assertIn(
            'readonly ros_snapshot_tool="${H1_FULL_ROS_SNAPSHOT_TOOL:-}"',
            cube_support,
        )
        self.assertIn('"${ros_snapshot_tool}" == /*', cube_support)
        self.assertIn('-f "${ros_snapshot_tool}"', cube_support)
        self.assertIn('-x "${ros_snapshot_tool}"', cube_support)
        self.assertIn("for attempt in 1 2 3", snapshot_reader)
        self.assertIn(
            'timeout 5 "${ros_snapshot_tool}" robot --timeout 3', snapshot_reader
        )
        self.assertNotIn("/usr/bin/python3 -c", cube_support)
        self.assertNotIn("import rclpy", cube_support)
        self.assertNotIn("JointState", cube_support)
        self.assertNotIn("ros2 topic echo", cube_support)
        self.assertIn("google.protobuf.Message", cube_support)
        self.assertIn('cube_pose_gate "DOOR_STAGE_CUBE_SPAWN_GATE"', full_wrapper)


class CubeCacheBuilderTest(unittest.TestCase):
    def test_legacy70_is_the_backward_compatible_default_variant(self) -> None:
        args = ASSET_TOOL.build_parser().parse_args(
            ["build-cube-cache", "/absolute/asset", "/absolute/cache"]
        )
        self.assertEqual(args.cache_variant, "legacy70")
        digest, stage_counts = ASSET_TOOL._left_rack_cache_variant_contract(
            args.cache_variant
        )
        self.assertEqual(digest, ASSET_TOOL.LEFT_RACK_Q60_8251_CACHE_SHA256)
        self.assertEqual(stage_counts, ASSET_TOOL.LEFT_RACK_Q60_8251_STAGE_COUNTS)

    def test_unknown_cache_variant_is_rejected_by_cli_and_handler(self) -> None:
        with redirect_stderr(StringIO()), self.assertRaises(SystemExit):
            ASSET_TOOL.build_parser().parse_args(
                [
                    "build-cube-cache",
                    "/absolute/asset",
                    "/absolute/cache",
                    "--cache-variant",
                    "unreviewed",
                ]
            )
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "asset.json"
            with self.assertRaisesRegex(ASSET_TOOL.AssetError, "unknown.*variant"):
                ASSET_TOOL.build_cube_cache_asset(
                    SimpleNamespace(
                        asset=str(output),
                        cache="/absolute/cache",
                        cache_variant="unreviewed",
                    )
                )
            self.assertFalse(output.exists())

    def test_cache_sha_cannot_be_supplied_or_overridden_on_cli(self) -> None:
        with redirect_stderr(StringIO()), self.assertRaises(SystemExit):
            ASSET_TOOL.build_parser().parse_args(
                [
                    "build-cube-cache",
                    "/absolute/asset",
                    "/absolute/cache",
                    "--cache-sha256",
                    "0" * 64,
                ]
            )

    def test_parallel90_registry_binds_reviewed_cache_identity(self) -> None:
        self.assertIn("parallel90", ASSET_TOOL.LEFT_RACK_CACHE_VARIANTS)
        self.assertEqual(
            ASSET_TOOL.LEFT_RACK_CACHE_VARIANTS["parallel90"]["sha256"],
            "dfc45b9ab0ad12e6a6e038ed0a2a3300838ec143ad8dd4c591b423273a352a02",
        )
        self.assertEqual(
            ASSET_TOOL.LEFT_RACK_CACHE_VARIANTS["parallel90"]["stage_counts"],
            {"approach": 32, "carry": 360, "support": 28, "retreat": 413},
        )

    def test_builder_is_deterministic_and_seals_the_six_message_split(self) -> None:
        joints = ASSET_TOOL.CONTROLLER_JOINTS[
            "/left_arm_controller/joint_trajectory"
        ]
        zero = [0.0 for _ in joints]
        paths = {
            stage: [list(zero), list(zero)]
            for stage in ASSET_TOOL.LEFT_RACK_Q60_8251_STAGE_COUNTS
        }
        cache_bundle = (
            paths,
            ASSET_TOOL.LEFT_RACK_CACHE_TASK_CONTRACTS["legacy70"],
        )
        with tempfile.TemporaryDirectory() as directory, mock.patch.object(
            ASSET_TOOL, "_read_left_rack_q60_8251_cache", return_value=cache_bundle
        ):
            first = Path(directory) / "first.json"
            second = Path(directory) / "second.json"
            with redirect_stdout(StringIO()):
                ASSET_TOOL.build_cube_cache_asset(
                    SimpleNamespace(asset=str(first), cache="/absolute/cache")
                )
                ASSET_TOOL.build_cube_cache_asset(
                    SimpleNamespace(asset=str(second), cache="/absolute/cache")
                )
            first_payload, first_durations, first_digest = ASSET_TOOL.load_asset(first)
            second_payload, second_durations, second_digest = ASSET_TOOL.load_asset(second)
        self.assertEqual(first_payload, second_payload)
        self.assertEqual(first_durations, second_durations)
        self.assertEqual(first_digest, second_digest)
        self.assertEqual(
            [message["topic"] for message in first_payload["messages"]],
            [
                "/left_arm_controller/joint_trajectory",
                "/left_gripper_controller/joint_trajectory",
                "/left_arm_controller/joint_trajectory",
                "/left_arm_controller/joint_trajectory",
                "/left_gripper_controller/joint_trajectory",
                "/left_arm_controller/joint_trajectory",
            ],
        )
        self.assertEqual(
            first_payload["messages"][1]["expected_actual_endpoint"]["positions"],
            list(ASSET_TOOL.LEFT_RACK_Q60_8251_CONTACT_ENDPOINT),
        )
        self.assertEqual(
            first_payload["messages"][1]["points"][-1]["positions"],
            list(ASSET_TOOL.LEFT_RACK_Q60_8251_COMMAND_ENDPOINT),
        )
        self.assertEqual(
            first_payload["messages"][4]["points"][0]["positions"],
            list(ASSET_TOOL.LEFT_RACK_Q60_8251_CONTACT_ENDPOINT),
        )
        contract = first_payload["cube_cache_contract"]
        self.assertEqual(contract["cache_variant"], "legacy70")
        self.assertEqual(
            contract["cache_sha256"], ASSET_TOOL.LEFT_RACK_Q60_8251_CACHE_SHA256
        )
        self.assertEqual(
            contract["target_pose_world"]["rpy_rad"],
            [0.0, 0.0, math.radians(70.0)],
        )

    def test_parallel90_builder_seals_full_pose_and_26_field_route(self) -> None:
        joints = ASSET_TOOL.CONTROLLER_JOINTS[
            "/left_arm_controller/joint_trajectory"
        ]
        zero = [0.0 for _ in joints]
        stage_counts = {
            "approach": 2,
            "carry": 2,
            "support": 2,
            "retreat": 2,
        }
        paths = {stage: [list(zero), list(zero)] for stage in stage_counts}
        digest = "a" * 64
        registry = {"sha256": digest, "stage_counts": stage_counts}
        cache_bundle = (
            paths,
            ASSET_TOOL.LEFT_RACK_CACHE_TASK_CONTRACTS["parallel90"],
        )
        with tempfile.TemporaryDirectory() as directory, mock.patch.dict(
            ASSET_TOOL.LEFT_RACK_CACHE_VARIANTS,
            {"parallel90": registry},
        ), mock.patch.object(
            ASSET_TOOL, "_read_left_rack_q60_8251_cache", return_value=cache_bundle
        ):
            output = Path(directory) / "parallel90.json"
            with redirect_stdout(StringIO()):
                ASSET_TOOL.build_cube_cache_asset(
                    SimpleNamespace(
                        asset=str(output),
                        cache="/absolute/cache",
                        cache_variant="parallel90",
                    )
                )
            payload, _durations, _payload_digest = ASSET_TOOL.load_asset(output)
            contract = payload["cube_cache_contract"]
            self.assertEqual(contract["cache_variant"], "parallel90")
            self.assertEqual(contract["cache_sha256"], digest)
            self.assertEqual(contract["stage_point_counts"], stage_counts)
            self.assertEqual(contract["object_size_xyz_m"], [0.08, 0.16, 0.03])
            self.assertEqual(
                contract["spawn_pose_world"],
                {
                    "xyz_m": [0.334, 0.377, 0.83],
                    "rpy_rad": [0.0, 0.0, math.pi / 2.0],
                },
            )
            self.assertEqual(
                contract["target_pose_world"],
                {
                    "xyz_m": [
                        0.610890542898,
                        -0.029527393333,
                        0.995407082351,
                    ],
                    "rpy_rad": [0.0, 0.0, math.pi / 2.0],
                },
            )
            route = contract["route_contract"]
            self.assertEqual(
                route["provenance_name"],
                "candidate3_cabinet_front_positive_x_parallel90_v1",
            )
            self.assertEqual(len(route["task_contract_values"]), 26)
            self.assertEqual(
                route["task_contract_values"][16:],
                [
                    32.0, 0.0, 75.0, 0.0, 0.0, 0.0, -90.0, 0.025,
                    0.0005, 0.0015,
                ],
            )
            self.assertEqual(route["carry_self_clearance_m"], 0.0005)
            self.assertEqual(route["support_release_height_m"], 0.0015)
            self.assertEqual(route["unscaled_arm_duration_ceiling_s"], 50.0)

            for field, replacement, error in (
                ("cache_sha256", "b" * 64, "not registered"),
                ("cache_variant", "legacy70", "not registered"),
            ):
                corrupted = dict(contract)
                corrupted[field] = replacement
                candidate = dict(payload)
                candidate["cube_cache_contract"] = corrupted
                with self.assertRaisesRegex(ASSET_TOOL.AssetError, error):
                    ASSET_TOOL._validate_payload(candidate)

            corrupted = json.loads(json.dumps(payload))
            corrupted["cube_cache_contract"]["target_pose_world"]["rpy_rad"][2] = 0.0
            with self.assertRaisesRegex(ASSET_TOOL.AssetError, "named semantic fields"):
                ASSET_TOOL._validate_payload(corrupted)

            corrupted = json.loads(json.dumps(payload))
            corrupted["cube_cache_contract"]["route_contract"][
                "unscaled_arm_duration_ceiling_s"
            ] = 35.0
            with self.assertRaisesRegex(ASSET_TOOL.AssetError, "named semantic fields"):
                ASSET_TOOL._validate_payload(corrupted)

            corrupted = json.loads(json.dumps(payload))
            corrupted["cube_cache_contract"]["route_contract"][
                "carry_self_clearance_m"
            ] = 0.003
            with self.assertRaisesRegex(ASSET_TOOL.AssetError, "named semantic fields"):
                ASSET_TOOL._validate_payload(corrupted)

            corrupted = json.loads(json.dumps(payload))
            corrupted["cube_cache_contract"]["route_contract"][
                "support_release_height_m"
            ] = 0.0
            with self.assertRaisesRegex(ASSET_TOOL.AssetError, "named semantic fields"):
                ASSET_TOOL._validate_payload(corrupted)

    def test_parallel90_task_contract_requires_exact_26_fields(self) -> None:
        expected = ASSET_TOOL.LEFT_RACK_CACHE_TASK_CONTRACTS["parallel90"]
        self.assertEqual(len(expected), 26)
        with self.assertRaisesRegex(ASSET_TOOL.AssetError, "25 fields, expected 26"):
            ASSET_TOOL._left_rack_task_contract("parallel90", expected[:-1])
        changed = list(expected)
        changed[12] = 70.0
        with self.assertRaisesRegex(ASSET_TOOL.AssetError, "field 13"):
            ASSET_TOOL._left_rack_task_contract("parallel90", changed)
        changed = list(expected)
        changed[24] = 0.003
        with self.assertRaisesRegex(ASSET_TOOL.AssetError, "field 25"):
            ASSET_TOOL._left_rack_task_contract("parallel90", changed)
        changed = list(expected)
        changed[25] = 0.0
        with self.assertRaisesRegex(ASSET_TOOL.AssetError, "field 26"):
            ASSET_TOOL._left_rack_task_contract("parallel90", changed)

    def test_cache_reader_rejects_a_nonbinding_file(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            cache = Path(directory) / "cache"
            cache.write_text("H1_LEFT_RACK_POSITION_CACHE_V1\n", encoding="utf-8")
            cache.chmod(0o400)
            with self.assertRaisesRegex(ASSET_TOOL.AssetError, "SHA-256 mismatch"):
                ASSET_TOOL._read_left_rack_q60_8251_cache(cache)

    def test_wrapper_binds_cube_split_and_default_schedules(self) -> None:
        source = FULL_WRAPPER_PATH.read_text(encoding="utf-8")
        self.assertIn("H1_CUBE_ATTACH_AFTER_INDEX:-2", source)
        self.assertIn("H1_CUBE_DETACH_AFTER_INDEX:-4", source)
        self.assertIn("count == 6 && CUBE_ATTACH_AFTER_INDEX == 2", source)
        for schedule in (
            "0:0.3,1:0.1,2:0.3,3:0.15,4:0.35,5:0.1",
            "6:0.12,7:0.15,8:0.1,9:0.1,10:0.1,11:0.1,12:0.1",
            "0:1,1:1",
            "2:0.75,3:0.75",
            "4:0.1,5:0.35",
        ):
            self.assertIn(schedule, source)
        self.assertIn('RECORD_FRAMERATE:-12', source)
        self.assertIn('window_tree="$(xwininfo -root -tree', source)
        self.assertNotIn(
            "xwininfo -root -tree 2>/dev/null | grep -Fq", source
        )
        neutral_gate = source.split("neutral_arms_and_open_hands_gate()", 1)[1].split(
            "left_clear_gate()", 1
        )[0]
        self.assertIn("left_shoulder_roll_joint 0.25 0.004", neutral_gate)
        self.assertIn('/usr/bin/python3 - "${FIRST_PUBLISH_FILE}"', source)
        self.assertIn("/usr/bin/python3 -c 'import time;", source)
        self.assertIn(
            "door+=0.0; handle+=0.0; door_velocity+=0.0; handle_velocity+=0.0",
            source,
        )
        self.assertIn('PHYSICS_MAX_STEP_SIZE:-0.008', source)
        self.assertIn('CUBE_CONTACT_PHYSICS_MAX_STEP_SIZE:-0.006', source)
        self.assertIn('CUBE_SUPPORTED_PHYSICS_MAX_STEP_SIZE:-0.006', source)
        self.assertIn('PHYSICS_REAL_TIME_FACTOR:-2.0', source)
        self.assertIn('CUBE_SUPPORTED_MINIMUM_RTF:-0.50', source)
        self.assertIn('CUBE_SUPPORTED_RTF_SAMPLE_COUNT:-3', source)
        self.assertIn('RTF_SAMPLE_COUNT:-1', source)
        self.assertIn('RTF_GATE_ATTEMPTS:-3', source)
        robot_snapshot = source.split("read_robot_joint_snapshot()", 1)[1].split(
            "joint_values_gate()", 1
        )[0]
        self.assertIn("for attempt in 1 2", robot_snapshot)
        self.assertIn('robot --timeout 3', robot_snapshot)
        incubator_snapshot = source.split("read_incubator_snapshot()", 1)[1].split(
            "mechanism_gate()", 1
        )[0]
        self.assertIn("for attempt in 1 2", incubator_snapshot)
        self.assertIn('incubator --timeout 3', incubator_snapshot)
        latch_reader = source.split("require_latch_state()", 1)[1].split(
            "read_robot_joint_snapshot()", 1
        )[0]
        self.assertIn("for attempt in 1 2 3", latch_reader)
        cube_reader = source.split("read_cube_pose()", 1)[1].split(
            "cube_pose_gate()", 1
        )[0]
        self.assertIn("for attempt in 1 2 3", cube_reader)
        self.assertIn(
            '"${stage}" == "cube" && "${suffix}" == "supported_carry"',
            source,
        )
        self.assertIn(
            '"${required_rtf}" "${required_count}"',
            source,
        )
        self.assertIn("run_parallel_gate_group HANDLE_STAGE", source)
        self.assertIn("run_replay_range cube 0 1 before_attach_approach", source)
        self.assertIn("CUBE_CONTACT_READY_STABILITY", source)
        self.assertIn('before_attach_close "1:1"', source)
        self.assertIn("[-0.01472, -0.01472]", source)
        self.assertIn("CUBE_PRE_ATTACH_CUBE_POSE_GATE", source)
        self.assertIn("CUBE_CONTACT_PHYSICS_APPLY_GATE", source)
        self.assertIn("CUBE_SUPPORTED_PHYSICS_APPLY_GATE", source)
        self.assertIn("CUBE_RETREAT_PHYSICS_RESTORE_GATE", source)
        self.assertIn("subscriber_args=(--minimum-subscriptions 2)", source)

    def test_wrapper_preloads_detached_supports_before_controller_timer(self) -> None:
        source = FULL_WRAPPER_PATH.read_text(encoding="utf-8")
        door_preload = source.split("\nrun_door_support_preload()\n", 1)[1].split(
            "\nrun_cube_support_preload()\n", 1
        )[0]
        cube_preload = source.split("\nrun_cube_support_preload()\n", 1)[1].split(
            "\nrun_door_support_hook()\n", 1
        )[0]
        door_attach = source.split("\nrun_door_support_hook()\n", 1)[1].split(
            "\nrun_cube_support_hook()\n", 1
        )[0]
        cube_attach_detach = source.split(
            "\nrun_cube_support_hook()\n", 1
        )[1].split("\nrun_robot_flow()\n", 1)[0]

        for helper_block in (
            door_preload,
            cube_preload,
            door_attach,
            cube_attach_detach,
        ):
            self.assertIn(
                'export H1_FULL_ROS_SNAPSHOT_TOOL="${ROS_SNAPSHOT_TOOL}"',
                helper_block,
            )
        self.assertEqual(
            source.count(
                'export H1_FULL_ROS_SNAPSHOT_TOOL="${ROS_SNAPSHOT_TOOL}"'
            ),
            4,
        )

        for marker in (
            'export H1_DOOR_ASSET="${ASSET_PATHS[door]}"',
            'export H1_DOOR_ASSET_SHA256="${ASSET_SHA256[door]}"',
            'export H1_DOOR_ATTACH_AFTER_INDEX="${DOOR_ATTACH_AFTER_INDEX}"',
            'export H1_DOOR_SUPPORT_MAX_FORCE="${DOOR_SUPPORT_MAX_FORCE}"',
            'export H1_DOOR_SUPPORT_MAX_TORQUE="${DOOR_SUPPORT_MAX_TORQUE}"',
        ):
            self.assertIn(marker, door_preload)
        self.assertIn('"${DOOR_SUPPORT_HELPER}" preload', door_preload)
        self.assertIn(
            'grep -Fqx "DOOR_SUPPORT_PRELOAD=PASS child=incubator::door_link '
            'force_cap=${DOOR_SUPPORT_MAX_FORCE} '
            'torque_cap=${DOOR_SUPPORT_MAX_TORQUE}"',
            door_preload,
        )

        for marker in (
            'export H1_CUBE_ASSET="${ASSET_PATHS[cube]}"',
            'export H1_CUBE_ASSET_SHA256="${ASSET_SHA256[cube]}"',
            'export H1_CUBE_ATTACH_AFTER_INDEX="${CUBE_ATTACH_AFTER_INDEX}"',
            'export H1_CUBE_DETACH_AFTER_INDEX="${CUBE_DETACH_AFTER_INDEX}"',
        ):
            self.assertIn(marker, cube_preload)
        self.assertIn('"${CUBE_SUPPORT_HELPER}" preload', cube_preload)
        self.assertIn(
            'grep -Fqx "CUBE_SUPPORT_PRELOAD=PASS" "${log_file}"',
            cube_preload,
        )

        flow = source.split("\nrun_robot_flow()\n", 1)[1].split(
            "\ncleanup()\n", 1
        )[0]
        first_fresh = flow.index("\n  fresh_scene_gate\n")
        door_preload_call = flow.index("\n  run_door_support_preload\n")
        cube_preload_call = flow.index("\n  run_cube_support_preload\n")
        preload_proof = flow.index(
            "\n  fresh_scene_gate\n", first_fresh + len("\n  fresh_scene_gate\n")
        )
        camera = flow.index("\n  set_recording_camera_pose\n")
        recorder = flow.index("\n  start_recorder\n")
        first_publish_monitor = flow.index("\n  start_first_publish_monitor\n")
        timer = flow.index('FLOW_INVOCATION_START_NS="$(monotonic_now_ns)"')
        first_replay = flow.index("\n  run_replay_range handle 0")
        self.assertLess(
            first_fresh,
            door_preload_call,
        )
        self.assertLess(door_preload_call, cube_preload_call)
        self.assertLess(cube_preload_call, preload_proof)
        self.assertLess(preload_proof, camera)
        self.assertLess(camera, recorder)
        self.assertLess(recorder, first_publish_monitor)
        self.assertLess(first_publish_monitor, timer)
        self.assertLess(timer, first_replay)
        self.assertIn("\n  run_door_support_hook\n", flow)
        self.assertIn("\n  run_cube_support_hook attach\n", flow)
        self.assertIn("\n  run_cube_support_hook detach\n", flow)

    def test_wrapper_closes_final_rack_gate_and_separates_video_acceptance(
        self,
    ) -> None:
        source = FULL_WRAPPER_PATH.read_text(encoding="utf-8")
        flow = source.split("\nrun_robot_flow()\n", 1)[1].split(
            "\ncleanup()\n", 1
        )[0]

        parallel_final = flow.index("\n  run_parallel_gate_group FINAL \\")
        direct_door = flow.index("\n  reject_direct_door_controller\n", parallel_final)
        rack_entry = flow.index(
            'cube_pose_gate "FINAL_SEQUENTIAL_CUBE_RACK_ENTRY_GATE"',
            direct_door,
        )
        rack_stability = flow.index(
            'cube_stability_gate "FINAL_SEQUENTIAL_CUBE_STABILITY_GATE"',
            rack_entry,
        )
        rack_exit = flow.index(
            'cube_pose_gate "FINAL_SEQUENTIAL_CUBE_RACK_EXIT_GATE"',
            rack_stability,
        )
        flow_end = flow.index('FLOW_END_NS="$(monotonic_now_ns)"', rack_exit)
        strict_first_publish_bound = flow.index(
            "elapsed_ns > 0 && elapsed_ns < MAX_WALL_SECONDS * 1000000000",
            flow_end,
        )
        strict_invocation_bound = flow.index(
            "invocation_elapsed_ns > 0 && "
            "invocation_elapsed_ns < MAX_WALL_SECONDS * 1000000000",
            strict_first_publish_bound,
        )
        stop_recorder = flow.index("\n  stop_recorder\n", strict_invocation_bound)
        verify_video = flow.index("\n  verify_video\n", stop_recorder)
        recorded_branch = flow.index(
            'if [[ "${RECORD_VIDEO}" == "1" ]]; then', verify_video
        )
        physical_pass = flow.index("FULL_ROBOT_FLOW_PHYSICAL=PASS", recorded_branch)
        diagnostic_branch = flow.index("\n  else\n", physical_pass)
        diagnostic_pass = flow.index(
            "FULL_ROBOT_FLOW_DIAGNOSTIC=PASS", diagnostic_branch
        )

        self.assertLess(parallel_final, direct_door)
        self.assertLess(direct_door, rack_entry)
        self.assertLess(rack_entry, rack_stability)
        self.assertLess(rack_stability, rack_exit)
        self.assertLess(rack_exit, flow_end)
        self.assertLess(flow_end, strict_first_publish_bound)
        self.assertLess(strict_first_publish_bound, strict_invocation_bound)
        self.assertLess(strict_invocation_bound, stop_recorder)
        self.assertLess(stop_recorder, verify_video)
        self.assertLess(verify_video, recorded_branch)
        self.assertLess(recorded_branch, physical_pass)
        self.assertLess(physical_pass, diagnostic_branch)
        self.assertLess(diagnostic_branch, diagnostic_pass)

        closure = flow[direct_door:flow_end]
        self.assertEqual(closure.count("0.610890542898"), 2)
        self.assertEqual(closure.count("-0.029527393333"), 2)
        self.assertEqual(closure.count("0.995407082351"), 2)
        self.assertEqual(closure.count("0.012 0.087266462599716"), 2)
        self.assertIn("video=${VIDEO_OUTPUT}", flow[physical_pass:diagnostic_branch])
        self.assertNotIn("video=${VIDEO_OUTPUT:-disabled}", flow)
        self.assertIn("video=disabled", flow[diagnostic_pass:])

    def test_door_support_preserves_checks_with_parallel_discovery(self) -> None:
        source = DOOR_SUPPORT_PATH.read_text(encoding="utf-8")
        self.assertIn("read_transport_infos()", source)
        self.assertIn('transport_info "${state_topic}" >', source)
        self.assertIn('transport_info "${attach_topic}" >', source)
        self.assertIn('transport_info "${detach_topic}" >', source)
        self.assertIn('local live_rows="${4:-}"', source)
        self.assertIn('pre_attach_robot_rows="$(<', source)
        self.assertIn('post_attach_robot_rows="$(read_robot_joint_rows)"', source)
        self.assertIn("pre_attach_mechanism_sample_count + 3", source)
        self.assertIn("mechanism_pulse_gate", source)
        self.assertIn("require_add_service_contract", source)
        self.assertIn("<initially_attached>false</initially_attached>", source)
        self.assertNotIn("for sample in 1 2 3; do", source)


class CloseReverseAssetTest(unittest.TestCase):
    def test_position_record_reverse_uses_t_minus_t_and_clears_derivatives(self) -> None:
        record = gripper_record(final_time=1.0, final_position=0.01)
        reversed_record = ASSET_TOOL._reverse_position_record(record, "test")
        self.assertEqual(reversed_record["topic"], record["topic"])
        self.assertEqual(reversed_record["points"][0]["positions"], [0.01, 0.01])
        self.assertEqual(reversed_record["points"][-1]["positions"], [0.0, 0.0])
        self.assertEqual(reversed_record["points"][0]["time_from_start"], duration(0.0))
        self.assertEqual(reversed_record["points"][-1]["time_from_start"], duration(1.0))
        for point in reversed_record["points"]:
            self.assertEqual(point["velocities"], [])
            self.assertEqual(point["accelerations"], [])
            self.assertEqual(point["effort"], [])

    def test_reverse_builder_parser_requires_sha_bound_recipe(self) -> None:
        args = ASSET_TOOL.build_parser().parse_args(
            [
                "build-close-reverse",
                "out.json",
                "/source.json",
                "--recipe",
                "door_close",
                "--expected-source-sha256",
                "a" * 64,
            ]
        )
        self.assertIs(args.handler, ASSET_TOOL.build_close_reverse_asset)
        self.assertEqual(args.recipe, "door_close")

    def test_close_scripts_keep_detach_at_q0_and_forbid_cleanup_mutation(self) -> None:
        detach = CLOSE_DETACH_PATH.read_text(encoding="utf-8")
        wrapper = CLOSE_WRAPPER_PATH.read_text(encoding="utf-8")
        self.assertIn("detach_after_index == 7", detach)
        self.assertIn(
            'parallel_detach_physical_gate "pre-detach" attached', detach
        )
        self.assertIn(
            'parallel_detach_physical_gate "post-detach" detached', detach
        )
        self.assertIn('mechanism_triplet_gate "${phase}"', detach)
        self.assertIn("DOOR_SUPPORT_DETACH_8253=PASS", detach)
        self.assertIn("run_replay door_close", wrapper)
        self.assertIn("run_replay handle_relock", wrapper)
        self.assertIn("CLOSE_RELOCK_8253_PHYSICAL=PASS", wrapper)
        self.assertNotIn("reset --", detach + wrapper)
        self.assertNotIn("simulation_direct_joint_trajectory\" -m", detach + wrapper)


class PositionOnlyCopyAssetTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.root = Path(self.temporary_directory.name)
        self.source = self.root / "source.json"
        self.output = self.root / "position-only.json"
        self.messages = cube_contact_messages()
        self.source_digest = write_sealed_asset(
            self.source, "cube", self.messages
        )

    def args(
        self,
        *,
        source: Path | str | None = None,
        output: Path | None = None,
        stage: str = "cube",
        digest: str | None = None,
    ) -> SimpleNamespace:
        return SimpleNamespace(
            asset=str(self.output if output is None else output),
            source=str(self.source if source is None else source),
            expected_stage=stage,
            expected_source_sha256=(
                self.source_digest if digest is None else digest
            ),
        )

    def test_parser_requires_stage_and_source_payload_binding(self) -> None:
        args = ASSET_TOOL.build_parser().parse_args(
            [
                "build-position-only-copy",
                "out.json",
                "/source.json",
                "--expected-stage",
                "cube",
                "--expected-source-sha256",
                "a" * 64,
            ]
        )
        self.assertIs(args.handler, ASSET_TOOL.build_position_only_copy_asset)
        self.assertEqual(args.expected_stage, "cube")
        self.assertEqual(args.expected_source_sha256, "a" * 64)

    def test_builds_exact_order_position_only_mode_0400_roundtrip(self) -> None:
        source_bytes = self.source.read_bytes()
        stdout = StringIO()
        with mock.patch.object(
            ASSET_TOOL, "load_asset", wraps=ASSET_TOOL.load_asset
        ) as load_asset, mock.patch.object(
            ASSET_TOOL,
            "_retimed_message_points",
            wraps=ASSET_TOOL._retimed_message_points,
        ) as retime, redirect_stdout(stdout):
            self.assertEqual(
                ASSET_TOOL.build_position_only_copy_asset(self.args()), 0
            )
            self.assertEqual(load_asset.call_count, 2)
            self.assertEqual(load_asset.call_args_list[0].args, (self.source,))
            self.assertEqual(load_asset.call_args_list[1].args, (self.output,))
            self.assertEqual(retime.call_count, len(self.messages))
            for index, call in enumerate(retime.call_args_list):
                self.assertEqual(call.args[1:3], (1.0, True))
                self.assertEqual(call.args[3], f"position-only message[{index}]")

        payload, durations, output_digest = ASSET_TOOL.load_asset(self.output)
        self.assertEqual(self.source.read_bytes(), source_bytes)
        self.assertEqual(self.output.stat().st_mode & 0o777, 0o400)
        self.assertEqual(self.output.stat().st_nlink, 1)
        self.assertEqual(payload["stage"], "cube")
        self.assertEqual(
            payload["controller_topics"], list(ASSET_TOOL.CONTROLLER_TOPICS)
        )
        self.assertEqual(len(durations), len(self.messages))
        for source_message, copied_message in zip(self.messages, payload["messages"]):
            self.assertEqual(copied_message["sequence"], source_message["sequence"])
            self.assertEqual(copied_message["topic"], source_message["topic"])
            self.assertEqual(
                copied_message["wall_offset_ns"], source_message["wall_offset_ns"]
            )
            self.assertEqual(
                copied_message["joint_names"], source_message["joint_names"]
            )
            self.assertEqual(
                copied_message.get("expected_actual_endpoint"),
                source_message.get("expected_actual_endpoint"),
            )
            for source_point, copied_point in zip(
                source_message["points"], copied_message["points"]
            ):
                self.assertEqual(copied_point["positions"], source_point["positions"])
                self.assertEqual(
                    copied_point["time_from_start"], source_point["time_from_start"]
                )
                self.assertEqual(copied_point["velocities"], [])
                self.assertEqual(copied_point["accelerations"], [])
                self.assertEqual(copied_point["effort"], [])

        lines = stdout.getvalue().splitlines()
        self.assertEqual(
            lines,
            [
                "BUILD_POSITION_ONLY_COPY_ASSET=PASS "
                f"stage=cube messages={len(self.messages)} "
                f"source_sha256={self.source_digest} sha256={output_digest} "
                f"path={self.output}"
            ],
        )

    def test_rejects_relative_source_wrong_stage_sha_and_mode(self) -> None:
        cases = (
            (
                self.args(source="source.json", output=self.root / "relative.json"),
                "source path must be absolute",
            ),
            (
                self.args(output=self.root / "stage.json", stage="handle"),
                "differs from expected stage",
            ),
            (
                self.args(output=self.root / "sha.json", digest="0" * 64),
                "differs from the explicit binding",
            ),
            (
                self.args(output=self.root / "malformed.json", digest="A" * 64),
                "64 lowercase hexadecimal",
            ),
        )
        for args, marker in cases:
            with self.subTest(marker=marker), self.assertRaisesRegex(
                ASSET_TOOL.AssetError, marker
            ):
                ASSET_TOOL.build_position_only_copy_asset(args)
            self.assertFalse(Path(args.asset).exists())

        self.source.chmod(0o600)
        with self.assertRaisesRegex(ASSET_TOOL.AssetError, "mode must be exactly 0400"):
            ASSET_TOOL.build_position_only_copy_asset(
                self.args(output=self.root / "mode.json")
            )
        self.assertFalse((self.root / "mode.json").exists())

    def test_existing_output_is_never_overwritten(self) -> None:
        with redirect_stdout(StringIO()):
            ASSET_TOOL.build_position_only_copy_asset(self.args())
        original = self.output.read_bytes()
        with self.assertRaisesRegex(ASSET_TOOL.AssetError, "target already exists"):
            ASSET_TOOL.build_position_only_copy_asset(self.args())
        self.assertEqual(self.output.read_bytes(), original)
        self.assertEqual(self.output.stat().st_mode & 0o777, 0o400)


class SafeMessageRetimingTest(unittest.TestCase):
    def test_rescales_time_velocity_and_acceleration_consistently(self) -> None:
        points = ASSET_TOOL._retimed_message_points(
            gripper_record(), 0.5, True, "trajectory[0]"
        )
        self.assertEqual(points[0]["time_ns"], 0)
        self.assertEqual(points[1]["time_ns"], 500_000_000)
        self.assertEqual(points[1]["velocities"], [0.04, 0.04])
        self.assertEqual(points[1]["accelerations"], [0.16, 0.16])
        self.assertEqual(points[1]["effort"], [3.0, 4.0])

    def test_rejects_position_velocity_and_acceleration_limit_violations(self) -> None:
        cases = (
            gripper_record(final_position=0.025),
            gripper_record(velocity=0.11),
            gripper_record(acceleration=0.11),
        )
        expected = ("position", "velocity", "acceleration")
        for record, marker in zip(cases, expected):
            with self.subTest(marker=marker), self.assertRaisesRegex(
                ASSET_TOOL.AssetError, marker
            ):
                ASSET_TOOL._retimed_message_points(record, 0.5, True, "trajectory[0]")

    def test_rejects_average_segment_velocity_when_derivatives_are_empty(self) -> None:
        record = gripper_record(final_time=0.1, final_position=0.02)
        for point in record["points"]:
            point["velocities"] = []
            point["accelerations"] = []
        with self.assertRaisesRegex(ASSET_TOOL.AssetError, "average velocity"):
            ASSET_TOOL._retimed_message_points(record, 0.5, True, "trajectory[0]")

    def test_rejects_retiming_that_collapses_monotonic_nanoseconds(self) -> None:
        record = gripper_record(final_time=0.000000001)
        with self.assertRaisesRegex(ASSET_TOOL.AssetError, "collapsed to zero"):
            ASSET_TOOL._retimed_message_points(record, 0.1, True, "trajectory[0]")

    def test_explicit_scale_one_still_opts_into_joint_limit_validation(self) -> None:
        with self.assertRaisesRegex(ASSET_TOOL.AssetError, "position"):
            ASSET_TOOL._retimed_message_points(
                gripper_record(final_position=0.025), 1.0, True, "trajectory[0]"
            )


class SegmentedReplayRangeTest(unittest.TestCase):
    def replay_args(self, stage: str, start: int | None, end: int | None):
        argv = ["replay", "asset.json", "--expected-stage", stage]
        if start is not None:
            argv.extend(("--start-index", str(start)))
        if end is not None:
            argv.extend(("--end-index", str(end)))
        return ASSET_TOOL.build_parser().parse_args(argv)

    def test_door_segment_uses_strict_global_half_open_range(self) -> None:
        args = self.replay_args("door", 1, 4)
        self.assertEqual(ASSET_TOOL._selected_replay_range(args, 5), (1, 4))

    def test_cube_segment_remains_supported(self) -> None:
        args = self.replay_args("cube", 0, 2)
        self.assertEqual(ASSET_TOOL._selected_replay_range(args, 3), (0, 2))

    def test_handle_and_left_clear_segments_remain_forbidden(self) -> None:
        for stage in ("handle", "left_clear"):
            with self.subTest(stage=stage), self.assertRaisesRegex(
                ASSET_TOOL.AssetError, "expected stage 'door' or 'cube'"
            ):
                ASSET_TOOL._selected_replay_range(self.replay_args(stage, 0, 1), 3)

    def test_door_segment_requires_both_bounds_and_strict_range(self) -> None:
        with self.assertRaisesRegex(ASSET_TOOL.AssetError, "requires both"):
            ASSET_TOOL._selected_replay_range(self.replay_args("door", 1, None), 5)
        for start, end in ((-1, 2), (1, 1), (3, 2), (1, 6)):
            with self.subTest(start=start, end=end), self.assertRaisesRegex(
                ASSET_TOOL.AssetError, "0 <= start < end <= 5"
            ):
                ASSET_TOOL._selected_replay_range(
                    self.replay_args("door", start, end), 5
                )

    def test_unsegmented_door_keeps_full_range(self) -> None:
        self.assertEqual(
            ASSET_TOOL._selected_replay_range(self.replay_args("door", None, None), 5),
            (0, 5),
        )


class OfflineRetimeCheckCommandTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary_directory.cleanup)
        self.root = Path(self.temporary_directory.name)

    def run_check(self, argv: list[str]) -> tuple[int, str, str]:
        output = StringIO()
        errors = StringIO()
        # A forbidden ROS import would fail this test immediately.  The
        # offline command must depend only on the sealed JSON and pure math.
        with mock.patch.dict(sys.modules, {"rclpy": None}), redirect_stdout(
            output
        ), redirect_stderr(errors):
            status = ASSET_TOOL.main(argv)
        return status, output.getvalue(), errors.getvalue()

    def test_success_prints_exactly_one_machine_readable_pass_record(self) -> None:
        asset = self.root / "handle.json"
        digest = write_sealed_asset(asset, "handle", [gripper_record()])
        status, output, errors = self.run_check(
            [
                "retime-check",
                str(asset),
                "--expected-stage",
                "handle",
                "--message-time-scales",
                "0:0.5",
            ]
        )
        self.assertEqual(status, 0)
        self.assertEqual(errors, "")
        lines = output.splitlines()
        self.assertEqual(len(lines), 1)
        self.assertEqual(lines[0].count("RETIME_CHECK=PASS"), 1)
        self.assertIn("stage=handle range=[0,1) messages=1 overrides=1", lines[0])
        self.assertIn("original_duration_seconds=1.000000000", lines[0])
        self.assertIn("retimed_duration_seconds=0.500000000", lines[0])
        self.assertIn("original_critical_path_seconds=1.000000000", lines[0])
        self.assertIn("retimed_critical_path_seconds=0.500000000", lines[0])
        self.assertTrue(lines[0].endswith(f"sha256={digest}"))

    def test_door_segment_is_validated_offline_with_global_indices(self) -> None:
        asset = self.root / "door.json"
        messages = [
            gripper_record(sequence=0),
            gripper_record(
                sequence=1,
                topic="/right_gripper_controller/joint_trajectory",
                wall_offset_ns=2_000_000_000,
            ),
        ]
        write_sealed_asset(asset, "door", messages)
        status, output, errors = self.run_check(
            [
                "retime-check",
                str(asset),
                "--expected-stage",
                "door",
                "--start-index",
                "1",
                "--end-index",
                "2",
                "--message-time-scales",
                "1:0.5",
            ]
        )
        self.assertEqual(status, 0)
        self.assertEqual(errors, "")
        self.assertIn("stage=door range=[1,2) messages=1 overrides=1", output)

    def test_stage_mismatch_and_joint_limit_failure_are_fail_closed(self) -> None:
        stage_asset = self.root / "stage.json"
        write_sealed_asset(stage_asset, "handle", [gripper_record()])
        status, output, errors = self.run_check(
            [
                "retime-check",
                str(stage_asset),
                "--expected-stage",
                "door",
                "--message-time-scales",
                "0:0.5",
            ]
        )
        self.assertEqual(status, 2)
        self.assertEqual(output, "")
        self.assertIn("stage mismatch", errors)

        limit_asset = self.root / "limit.json"
        write_sealed_asset(
            limit_asset, "handle", [gripper_record(acceleration=0.11)]
        )
        status, output, errors = self.run_check(
            [
                "retime-check",
                str(limit_asset),
                "--expected-stage",
                "handle",
                "--message-time-scales",
                "0:0.5",
            ]
        )
        self.assertEqual(status, 2)
        self.assertEqual(output, "")
        self.assertIn("retimed acceleration", errors)

    def test_message_time_scales_is_required_and_may_not_repeat(self) -> None:
        with redirect_stderr(StringIO()), self.assertRaises(SystemExit):
            ASSET_TOOL.build_parser().parse_args(
                ["retime-check", "asset.json", "--expected-stage", "handle"]
            )
        errors = StringIO()
        with redirect_stderr(errors):
            status = ASSET_TOOL.main(
                [
                    "retime-check",
                    "unused.json",
                    "--expected-stage",
                    "handle",
                    "--message-time-scales",
                    "0:0.5",
                    "--message-time-scales",
                    "1:0.5",
                ]
            )
        self.assertEqual(status, 2)
        self.assertIn("may be supplied exactly once", errors.getvalue())


class RetimeTimelineAndLoggingTest(unittest.TestCase):
    def setUp(self) -> None:
        self.messages = [
            gripper_record(sequence=0, wall_offset_ns=0, final_time=1.0),
            gripper_record(
                sequence=1,
                topic="/right_gripper_controller/joint_trajectory",
                wall_offset_ns=500_000_000,
                final_time=1.0,
            ),
        ]
        self.durations = [1_000_000_000, 1_000_000_000]

    def test_independent_shortening_changes_critical_path_without_reordering(self) -> None:
        original = ASSET_TOOL._replay_overlap_groups(
            self.messages, self.durations, 0, 2, 1.0, 0.02
        )
        retimed = ASSET_TOOL._replay_overlap_groups(
            self.messages, self.durations, 0, 2, 1.0, 0.02, {0: 0.25}
        )
        self.assertEqual(ASSET_TOOL._replay_groups_critical_duration_ns(original), 1_500_000_000)
        self.assertEqual(ASSET_TOOL._replay_groups_critical_duration_ns(retimed), 1_250_000_000)
        self.assertEqual([item["index"] for group in retimed for item in group], [0, 1])

    def test_plan_log_lists_each_message_and_totals(self) -> None:
        output = StringIO()
        with redirect_stdout(output):
            ASSET_TOOL._log_message_retime_plan(
                self.messages, self.durations, 0, 2, {0: 0.25}, 0.02
            )
        text = output.getvalue()
        self.assertIn(
            "REPLAY_RETIME index=0 topic=/left_gripper_controller/joint_trajectory "
            "override=1 scale=0.25 original_duration_seconds=1.000000000 "
            "retimed_duration_seconds=0.250000000",
            text,
        )
        self.assertIn("REPLAY_RETIME index=1", text)
        self.assertIn("override=0 scale=1", text)
        self.assertIn("REPLAY_RETIME_TOTAL=PASS messages=2", text)
        self.assertIn("original_duration_seconds=2.000000000", text)
        self.assertIn("retimed_duration_seconds=1.250000000", text)
        self.assertIn("original_critical_path_seconds=1.500000000", text)
        self.assertIn("retimed_critical_path_seconds=1.250000000", text)


if __name__ == "__main__":
    unittest.main()
