"""ROS-free tests for the fail-closed Candidate 3 live certificate 829."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sqlite3
import sys
from types import ModuleType, SimpleNamespace

import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
SOURCE = (WORKSPACE / "src/h1_2_moveit_demo/scripts/"
          "h1_candidate3_live_certificate_829.py")
MODULE = ModuleType("h1_candidate3_live_certificate_829_test")
MODULE.__file__ = str(SOURCE)
sys.modules[MODULE.__name__] = MODULE
exec(compile(SOURCE.read_bytes(), str(SOURCE), "exec"), MODULE.__dict__)


def _canonical(value):
    return (json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
        allow_nan=False).encode("utf-8") + b"\n")


def _sealed(path, value, mode=0o400):
    raw = value if type(value) is bytes else _canonical(value)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(raw)
    path.chmod(mode)
    return hashlib.sha256(raw).hexdigest()


def _right_authorization_value():
    return {
        "schema": MODULE.RIGHT_EXECUTION_AUTHORIZATION_SCHEMA,
        "controller_execution_authorized": True,
        "execution_ready": False,
        "live_execution_passed": False,
        "live_result": None,
        "controller_commands_issued": 0,
        "offline_only": True,
        "bindings": {},
        "controller_topics": [],
        "stage_order": {"prefix": [], "suffix": []},
        "limits": {},
        "auditor_version": "synthetic",
        "execution_topology": "synthetic",
    }


def _argv_sha(argv):
    return hashlib.sha256(
        b"".join(item.encode("utf-8") + b"\0" for item in argv)).hexdigest()


class JsonCdrDecoder:
    """Test double: payloads use JSON while exercising the SQLite boundary."""

    def deserialize(self, type_name, payload):
        del type_name
        return json.loads(payload.decode("utf-8"))


def _payload(value):
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"),
        allow_nan=False).encode("utf-8")


def _trajectory(topic):
    positions = [0.0] * len(MODULE.CONTROLLER_JOINTS[topic])
    return {
        "joint_names": list(MODULE.CONTROLLER_JOINTS[topic]),
        "points": [{
            "positions": positions,
            "time_from_start_ns": 250_000_000,
        }],
    }


def _scheduled_middle_trajectory(
        trajectory, speed_scale=0.5,
        lead_ns=MODULE.MIDDLE_EXECUTION_LEAD_NS):
    value = json.loads(json.dumps(trajectory))
    for point in value["points"]:
        point["time_from_start_ns"] = lead_ns + int(round(
            point["time_from_start_ns"] / speed_scale))
    return value


def _create_bag(path, marker_values, left_gripper_trajectories):
    connection = sqlite3.connect(path)
    connection.execute(
        "CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT NOT NULL, "
        "type TEXT NOT NULL, serialization_format TEXT NOT NULL, "
        "offered_qos_profiles TEXT NOT NULL)")
    connection.execute(
        "CREATE TABLE messages (id INTEGER PRIMARY KEY, "
        "topic_id INTEGER NOT NULL, timestamp INTEGER NOT NULL, "
        "data BLOB NOT NULL)")
    topic_ids = {}
    for topic_id, (name, type_name) in enumerate(
            MODULE.TOPIC_TYPES.items(), start=1):
        topic_ids[name] = topic_id
        connection.execute(
            "INSERT INTO topics VALUES (?,?,?,?,?)",
            (topic_id, name, type_name, "cdr", ""))
    messages = []
    message_id = 1
    marker_by_name = {item["name"]: item for item in marker_values}
    marker_start = marker_values[0]["bag_timestamp_ns"]
    marker_end = marker_values[-1]["bag_timestamp_ns"]
    telemetry_start = marker_start - 50_000_000
    telemetry_end = marker_end + 50_000_000
    sim_start = 100_000_000_000
    timestamp = telemetry_start
    native_sample_index = 0
    while timestamp <= telemetry_end:
        sim_timestamp = (
            sim_start + native_sample_index * MODULE.NATIVE_SIM_PERIOD_NS)
        messages.append((
            message_id, topic_ids["/clock"], timestamp,
            sqlite3.Binary(_payload({"clock_ns": sim_timestamp}))))
        message_id += 1
        messages.append((
            message_id, topic_ids["/incubator/joint_states"], timestamp,
            sqlite3.Binary(_payload({
                "header_stamp_ns": sim_timestamp,
                "names": list(MODULE.INCUBATOR_JOINTS),
                "positions": [0.0, math.pi / 2.0],
            }))))
        message_id += 1
        if sim_timestamp % MODULE.ROBOT_JOINT_PERIOD_NS == 0:
            robot_sample_index = (
                native_sample_index * MODULE.NATIVE_SIM_PERIOD_NS //
                MODULE.ROBOT_JOINT_PERIOD_NS)
            positions = []
            for name in MODULE.ALL_ROBOT_JOINTS:
                if name == "torso_joint":
                    positions.append(0.5)
                elif name == "left_hip_yaw_joint":
                    positions.append(robot_sample_index * 0.001)
                elif name == "right_ankle_roll_joint":
                    positions.append(robot_sample_index * -0.0005)
                else:
                    positions.append(0.0)
            messages.append((
                message_id, topic_ids["/joint_states"], timestamp,
                sqlite3.Binary(_payload({
                    "header_stamp_ns": sim_timestamp,
                    "names": list(MODULE.ALL_ROBOT_JOINTS),
                    "positions": positions,
                }))))
            message_id += 1
        timestamp += MODULE.NATIVE_SIM_PERIOD_NS
        native_sample_index += 1

    for marker in marker_values:
        marker_payload = {
            "schema": MODULE.MARKER_PAYLOAD_SCHEMA,
            "run_id": marker["run_id"],
            "sequence": marker["sequence"],
            "monotonic_ns": marker["monotonic_ns"],
            "process_role": marker["process_role"],
            "name": marker["name"],
            "line": marker["line"],
        }
        messages.append((
            message_id, topic_ids[MODULE.MARKER_TOPIC],
            marker["bag_timestamp_ns"],
            sqlite3.Binary(_payload({
                "data": _canonical(marker_payload)[:-1].decode("ascii"),
            }))))
        message_id += 1

    def midpoint(left, right):
        return (marker_by_name[left]["bag_timestamp_ns"] +
                marker_by_name[right]["bag_timestamp_ns"]) // 2

    topic_times = {
        "/torso_controller/joint_trajectory": [
            marker_start + 30_000_000 + i * 35_000_000 for i in range(8)],
        "/right_arm_controller/joint_trajectory": [
            marker_start + 32_000_000 + i * 35_000_000 for i in range(8)],
        "/left_arm_controller/joint_trajectory": [
            marker_start + 95_000_000 + i * 20_000_000 for i in range(8)],
        "/left_gripper_controller/joint_trajectory": [
            marker_by_name["Q90_BEFORE_MIDDLE"]["bag_timestamp_ns"] +
            1_000_000,
            marker_by_name["Q90_BEFORE_MIDDLE"]["bag_timestamp_ns"] +
            2_000_000,
        ],
        "/right_gripper_controller/joint_trajectory": [
            midpoint("PREFIX_INGRESS_OPEN_TO_Q0",
                     "PREFIX_CLOSE_Q0_RIGHT_HAND"),
            midpoint("PREFIX_CLOSE_Q0_RIGHT_HAND",
                     "PREFIX_CONTACT_ARC_Q0_Q90"),
            midpoint("Q90_AFTER_MIDDLE", "SUFFIX_CONTACT_ARC_Q90_Q0"),
            midpoint("SUFFIX_CONTACT_ARC_Q90_Q0",
                     "SUFFIX_RELEASE_Q0_RIGHT_HAND"),
        ],
    }
    for topic, timestamps in topic_times.items():
        for index, command_timestamp in enumerate(timestamps):
            trajectory = (_trajectory(topic) if topic !=
                          "/left_gripper_controller/joint_trajectory" else
                          left_gripper_trajectories[index])
            messages.append((
                message_id, topic_ids[topic], command_timestamp,
                sqlite3.Binary(_payload(trajectory))))
            message_id += 1
    connection.executemany(
        "INSERT INTO messages VALUES (?,?,?,?)", messages)
    connection.commit()
    connection.close()
    path.chmod(0o400)


def _machine_line(prefix, fields):
    return prefix + " " + " ".join(
        f"{key}={value}" for key, value in fields) + "\n"


def _fake_fcl_stdout(telemetry, geometry_sha, telemetry_sha):
    lines = []
    pad1_count = 0
    pad2_count = 0
    contact_count = 0
    for index, row in enumerate(telemetry.rows):
        required = row["contact_required"]
        contact_count += int(required)
        pad1 = required and index % 3 != 0
        pad2 = required and index % 2 == 0
        pad1_count += int(pad1)
        pad2_count += int(pad2)
        pad2_only_grip = required and index % 5 == 0
        pad1_distance = (
            0.00035 if pad2_only_grip else
            0.0002 if required else 0.0004)
        pad2_distance = 0.0002 if pad2_only_grip else 0.0005
        values = {
            "id": row["id"],
            "status": "PASS",
            "valid": "true",
            "contact_required": str(required).lower(),
            "door_angle_deg": MODULE._decimal(math.degrees(
                row["door_angle_rad"])),
            "handle_angle_deg": MODULE._decimal(math.degrees(
                row["handle_angle_rad"])),
            "torso_rad": MODULE._decimal(
                row["positions"]["torso_joint"]),
            "collision_free": "true",
            "pad1_patch_valid": str(pad1).lower(),
            "pad2_patch_valid": str(pad2).lower(),
            "right_pad_grip_engaged": str(required).lower(),
            "pad1_contacts": "4" if pad1 else "0",
            "pad2_contacts": "4" if pad2 else "0",
            "pad1_handle_distance_m": MODULE._decimal(pad1_distance),
            "pad2_handle_distance_m": MODULE._decimal(pad2_distance),
            "nonpad_handle_clearance_m": "0.00069999999999999999",
            "adjacent_finger_handle_clearance_m":
                "0.000010000000000000001",
            "left_handle_clearance_m": "0.050999999999999997",
            "patch_span_m": "0.0070000000000000001" if required else "0",
            "patch_station_m": "0.022499999999999999" if required else "0",
            "patch_radius_m": "0.044" if required else "0",
            "geometric_only": "true",
            "force_or_wrench_claimed": "false",
        }
        lines.append(_machine_line(
            "RIGHT_HANDLE_LIVE_FCL_ROW_829",
            [(key, values[key]) for key in MODULE.FCL_ROW_KEYS]))
    evaluated = len(telemetry.rows)
    noncontact = evaluated - contact_count
    summary = {
        "status": "PASS",
        "valid": "true",
        "evaluated": str(evaluated),
        "passed": str(evaluated),
        "contact_required": str(contact_count),
        "contact_required_passed": str(contact_count),
        "noncontact": str(noncontact),
        "noncontact_passed": str(noncontact),
        "pad1_patch_valid": str(pad1_count),
        "pad2_patch_valid": str(pad2_count),
        "right_pad_grip_engaged": str(contact_count),
        "max_torso_rad": "0.5",
        "max_contact_minimum_right_pad_distance_m":
            "0.00020000000000000001",
        "min_nonpad_handle_clearance_m": "0.00069999999999999999",
        "min_adjacent_finger_handle_clearance_m":
            "0.000010000000000000001",
        "min_left_handle_clearance_m": "0.050999999999999997",
        "min_patch_span_m": "0.0070000000000000001",
        "min_patch_station_m": "0.022499999999999999",
        "min_patch_radius_m": "0.044",
        "collision_free": "true",
        "scene_sha256": geometry_sha["scene"],
        "path_sha256": geometry_sha["path"],
        "telemetry_sha256": telemetry_sha,
        "urdf_sha256": geometry_sha["urdf"],
        "srdf_sha256": geometry_sha["srdf"],
        "handle_stl_sha256": geometry_sha["handle"],
        "reason": "NONE",
        "geometric_only": "true",
        "force_or_wrench_claimed": "false",
    }
    lines.append(_machine_line(
        "RIGHT_HANDLE_LIVE_FCL_SUMMARY_829",
        [(key, summary[key]) for key in MODULE.FCL_SUMMARY_KEYS]))
    return "".join(lines).encode("ascii"), {
        "contact": contact_count,
        "grip": contact_count,
        "pad1": pad1_count,
        "pad2": pad2_count,
        "maximum_contact_minimum_right_pad_distance_m": 0.0002,
        "minimum_adjacent_finger_handle_clearance_m": 0.00001,
        "minimum_buffered_nonpad_handle_clearance_m": 0.0007,
    }


def _fixture(tmp_path):
    root = (tmp_path / "run").resolve()
    root.mkdir(mode=0o700, parents=True)
    evidence = root / "evidence"
    evidence.mkdir(mode=0o700)
    output = root / "output"
    output.mkdir(mode=0o700)
    run_id = "8" * 64

    paths = SimpleNamespace(
        plan=evidence / "live_plan_829.json",
        bag=evidence / "live_829.db3",
        markers=evidence / "markers_829.jsonl",
        proc=evidence / "proc_snapshot_829.json",
        video_audit=evidence / "video_audit_829.json",
        postflight=evidence / "postflight_829.json",
        workcell=evidence / "workcell_pose_audit_829.json",
        camera=evidence / "gui_camera_audit_829.json",
        window=evidence / "video_window_audit_829.json",
        formal=evidence / "formal_authorities.json",
        replay=evidence / "exact_fcl_replay_829.json",
        telemetry_tsv=evidence / "exact_fcl_telemetry_829.tsv",
        contact_path=evidence / "geometry" / "contact_path_v4.json",
        handle_stl=evidence / "geometry" / "incubator_handle.STL",
        scene_cdr=evidence / "geometry" / "scene.cdr",
        urdf=evidence / "geometry" / "robot.urdf",
        srdf=evidence / "geometry" / "robot.srdf",
        replay_source=evidence / "auditor" / "live_replay.cpp",
        replay_path_base_source=evidence / "auditor" / "path_base.cpp",
        replay_middle_base_source=evidence / "auditor" / "middle_base.cpp",
        replay_binary=evidence / "auditor" / "live_replay",
        replay_stdout=evidence / "auditor" / "live_replay.stdout",
        replay_stderr=evidence / "auditor" / "live_replay.stderr",
        supervisor_source=evidence / "tools" / "live_supervisor.py",
        video=evidence / "candidate3_829.mp4",
        certificate=output / "candidate3_live_certificate_829.json",
    )

    def bound(path, digest, mode="0400"):
        return {
            "path": str(path),
            "sha256": digest,
            "size_bytes": path.stat().st_size,
            "mode": mode,
        }

    geometry_sha = {
        "path": _sealed(paths.contact_path, b"synthetic v4 contact path\n"),
        "handle": _sealed(paths.handle_stl, b"solid handle\nendsolid handle\n"),
        "scene": _sealed(paths.scene_cdr, b"synthetic exact scene cdr"),
        "urdf": _sealed(paths.urdf, b"<robot name='h1_2'/>\n"),
        "srdf": _sealed(paths.srdf, b"<robot name='h1_2'/>\n"),
    }
    replay_source_sha = _sealed(
        paths.replay_source, b"// exact FCL live replay source\n")
    replay_path_base_source_sha = _sealed(
        paths.replay_path_base_source, b"// exact path-base source\n")
    replay_middle_base_source_sha = _sealed(
        paths.replay_middle_base_source, b"// exact middle-base source\n")
    replay_binary_sha = _sealed(
        paths.replay_binary, b"synthetic exact FCL binary", 0o500)
    replay_stderr_sha = _sealed(paths.replay_stderr, b"")

    supervisor_sha = _sealed(
        paths.supervisor_source, b"#!/usr/bin/python3\n# supervisor\n", 0o500)
    certificate_sha = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    assert SOURCE.stat().st_mode & 0o777 == 0o500

    formal_paths = {
        identifier: evidence / "formal" / f"{index:02d}_{identifier}.json"
        for index, identifier in enumerate(MODULE.FORMAL_AUTHORITY_IDS)
    }
    right_auth_sha = _sealed(
        formal_paths["right_execution_authorization"],
        _right_authorization_value())
    canonical_e0 = {
        "right_arm_rad": [-0.41, 0.23, 1.10, 0.87, -0.30, 0.40, -0.99],
        "right_fingers_m": {
            "right_dex1_finger_1_joint": 0.0108,
            "right_dex1_finger_2_joint": 0.007,
        },
        "torso_yaw_rad": math.radians(-2.7349115403864994),
    }
    left_close = _trajectory(
        "/left_gripper_controller/joint_trajectory")
    left_close["points"] = [
        {"positions": [0.0, 0.0], "time_from_start_ns": 50_000_000},
        {"positions": [0.005, 0.005], "time_from_start_ns": 250_000_000},
        {"positions": [0.0105, 0.0105],
         "time_from_start_ns": 500_000_000},
    ]
    left_open = _trajectory(
        "/left_gripper_controller/joint_trajectory")
    left_open["points"] = [
        {"positions": [0.0105, 0.0105],
         "time_from_start_ns": 50_000_000},
        {"positions": [-0.004, -0.004],
         "time_from_start_ns": 200_000_000},
        {"positions": [-0.019, -0.019],
         "time_from_start_ns": 400_000_000},
    ]
    support_contract = {
        "schema": "h1_support_contract_828/v1",
        "boundaries": {"CUBE_RELEASE": {"cube_pose": {
            "position": [0.611, -0.030, 0.995],
            "orientation": [
                0.0, 0.0, 0.7071067811865475,
                0.7071067811865476],
        }}},
    }
    support_contract_sha = MODULE._canonical_hash(support_contract)
    middle_plan_value = {
        "schema": "h1_candidate3_middle_plan/828-e0-v1",
        "offline_only": True,
        "controller_commands_issued": 0,
        "support_contract_sha256": support_contract_sha,
        "phases": [
            {"name": "cube_contact", "commands": [{
                "topic": "/left_gripper_controller/joint_trajectory",
                **left_close,
            }]},
            {"name": "cube_release", "commands": [{
                "topic": "/left_gripper_controller/joint_trajectory",
                **left_open,
            }]},
        ],
    }
    middle_plan_sha = _sealed(
        formal_paths["middle_plan"], middle_plan_value)
    projected_e0 = {"label": "canonical_e0_q90_held", **canonical_e0}
    contact_windows = [
        {"id": "right_handle_pad_contact", "valid": True,
         "authorized_sample_count": 8},
        {"id": "left_cube_grasp_carry", "valid": True,
         "authorized_sample_count": 8,
         "object_ids": ["green_cube_dynamic_exact",
                        "green_cube_10mm_guard"],
         "phase_bindings": [
             {"phase_name": "cube_contact"},
             {"phase_name": "cube_carry_place"},
             {"phase_name": "cube_release"}]},
        {"id": "left_cube_release_retreat", "valid": True,
         "authorized_sample_count": 4,
         "object_ids": ["green_cube_dynamic_exact",
                        "green_cube_10mm_guard"],
         "phase_bindings": [{"phase_name": "cube_retreat"}]},
    ]
    middle_result_value = {
        "schema": "h1_candidate3_middle_exact_result/828-e0-v1",
        "status": "PASS", "valid": True, "offline_only": True,
        "controller_commands_issued": 0,
        "plan_file_sha256": middle_plan_sha,
        "support_contract_sha256": support_contract_sha,
        "scene_binding": {
            "required_object_ids": [
                "environment_table_10mm_margin", "green_cube_10mm_guard",
                "green_cube_dynamic_exact", "incubator_body_exact",
                "incubator_door_exact", "incubator_handle_exact"],
            "scene_cdr_sha256": geometry_sha["scene"],
            "planning_frame": "world",
        },
        "contact_authorizations": {
            "schema":
                "h1_candidate3_middle_contact_authorizations/828-e0-v1",
            "windows": contact_windows,
            "unauthorized_contacts": {"count": 0, "pairs": []},
            "valid": True,
        },
        "entry_projection": {
            "source_prefix": {"endpoint_label": "canonical_e0_q90_held"},
            "projected_handle_angle_rad": math.pi / 2.0,
            "projected_e0": projected_e0,
        },
        "terminal_seam": {
            "entry_e0": dict(canonical_e0),
            "return_e0": dict(canonical_e0),
        },
    }
    middle_result_sha = _sealed(
        formal_paths["middle_exact_result"], middle_result_value)
    middle_certificate_sha = _sealed(formal_paths["middle_certificate"], {
        "schema": "h1_candidate3_middle_certificate/828-e0-v1",
        "status": "PASS", "valid": True, "offline_only": True,
        "controller_commands_issued": 0,
        "exact_result_sha256": middle_result_sha,
        "plan_sha256": middle_plan_sha,
        "support_contract_sha256": support_contract_sha,
    })
    _sealed(formal_paths["middle_support_manifest"], {
        "schema": "h1_support_orchestration_828/v1",
        "controller_execution_authorized": True,
        "live_execution_passed": False,
        "bindings": {
            "plan_schema": "h1_candidate3_middle_plan/828-e0-v1",
            "plan_sha256": middle_plan_sha,
            "exact_result_schema":
                "h1_candidate3_middle_exact_result/828-e0-v1",
            "exact_result_sha256": middle_result_sha,
            "certificate_schema":
                "h1_candidate3_middle_certificate/828-e0-v1",
            "certificate_sha256": middle_certificate_sha,
            "support_contract_sha256": support_contract_sha,
        },
        "support_contract": support_contract,
    })
    _sealed(formal_paths["right_manifest"], {
        "audited_state_identity": {
            "q90_canonical_e0": {
                "handle_angle_deg": 90.0,
                "torso_yaw_deg": -2.7349115403864994,
                "right_pad_slider_mm": [10.8, 7.0],
                "right_arm_rad": canonical_e0["right_arm_rad"],
            },
            "endpoint_rows_unchanged_from_8263": True,
        },
        "sealed_bindings": {
            "controller_execution_authorization": {
                "path": str(formal_paths[
                    "right_execution_authorization"]),
                "sha256": right_auth_sha,
                "mode_octal": "0400", "nlink": 1,
                "schema":
                    "h1_right_handle_controller_execution_authorization/v1",
            },
            "contact_path": {
                "path": str(paths.contact_path),
                "sha256": geometry_sha["path"],
                "mode_octal": "0400", "nlink": 1,
                "schema": "h1_moveit/right_handle_variable_torso_path/v4",
            },
        },
    })
    already_written = {
        "right_execution_authorization", "right_manifest", "middle_plan",
        "middle_exact_result", "middle_certificate",
        "middle_support_manifest",
    }
    executable_authorities = {
        "right_executor_source", "middle_executor_source", "flow_source",
        "support_source", "gazebo_launch_source",
    }
    for identifier in MODULE.FORMAL_AUTHORITY_IDS:
        if identifier not in already_written:
            if identifier == "telemetry_bridge_config":
                raw = (WORKSPACE / "src/h1_2_moveit_config/config/"
                       "candidate3_telemetry_bridge_829.yaml").read_bytes()
            elif identifier == "rosbag_qos_overrides":
                raw = (WORKSPACE / "runtime_assets/"
                       "candidate3_rosbag_qos_overrides_829.yaml").read_bytes()
            else:
                raw = f"formal:{identifier}\n".encode("ascii")
            _sealed(
                formal_paths[identifier],
                raw,
                0o500 if identifier in executable_authorities else 0o400)
    authority_items = []
    authority_by_id = {}
    for identifier in MODULE.FORMAL_AUTHORITY_IDS:
        path = formal_paths[identifier]
        mode = path.stat().st_mode & 0o777
        item = {
            "id": identifier,
            "path": str(path),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "size_bytes": path.stat().st_size,
            "mode": f"{mode:04o}",
        }
        authority_items.append(item)
        authority_by_id[identifier] = item
    formal_sha = _sealed(paths.formal, {
        "schema": MODULE.FORMAL_SCHEMA,
        "run_id": run_id,
        "authorities": authority_items,
        "valid": True,
        "status": "PASS",
    })

    def authority_flags(identifier, path_flag, sha_flag):
        item = authority_by_id[identifier]
        return [path_flag, item["path"], sha_flag, item["sha256"]]

    launch_argv = [
        "/usr/bin/python3", str(paths.supervisor_source), "_launch-child",
        "--self-sha256", supervisor_sha, "--plan", str(paths.plan),
    ]
    for identifier, path_flag, sha_flag in (
        ("gazebo_launch_source", "--gazebo-launch-source",
         "--gazebo-launch-source-sha256"),
        ("gazebo_world_sdf", "--gazebo-world-sdf",
         "--gazebo-world-sdf-sha256"),
        ("incubator_model_sdf", "--incubator-model-sdf",
         "--incubator-model-sdf-sha256"),
        ("robot_gazebo_control_xacro", "--robot-gazebo-control-xacro",
         "--robot-gazebo-control-xacro-sha256"),
        ("ros2_controllers_config", "--ros2-controllers-config",
         "--ros2-controllers-config-sha256"),
        ("robot_joint_state_relay_source",
         "--robot-joint-state-relay-source",
         "--robot-joint-state-relay-source-sha256"),
        ("telemetry_bridge_config", "--telemetry-bridge-config",
         "--telemetry-bridge-config-sha256"),
    ):
        launch_argv += authority_flags(identifier, path_flag, sha_flag)
    launch_argv += [
        "spawn_incubator:=true", "table_x:=0.600",
        "incubator_x:=0.590", "incubator_y:=0.0",
        "incubator_z:=0.815", "cube_x:=0.334",
        "bridge_support_states:=false",
        "robot_joint_state_relay_source:=" +
        authority_by_id["robot_joint_state_relay_source"]["path"],
        "robot_joint_state_relay_sha256:=" +
        authority_by_id["robot_joint_state_relay_source"]["sha256"],
        "telemetry_bridge_config:=" +
        authority_by_id["telemetry_bridge_config"]["path"],
        "telemetry_bridge_config_sha256:=" +
        authority_by_id["telemetry_bridge_config"]["sha256"],
    ]
    video_argv = [
        "/usr/bin/python3", str(paths.supervisor_source), "_video-child",
        "--self-sha256", supervisor_sha, "--plan", str(paths.plan),
        "--run-id", run_id, "--window-audit", str(paths.window),
        "--crop", "1160:1530:0:80", "--framerate", "12",
        "--crf", "20", "--xwininfo", "/usr/bin/xwininfo",
        "--ffmpeg", "/usr/bin/ffmpeg", "--output", str(paths.video),
    ]
    recorder_argv = [
        "/opt/ros/jazzy/bin/ros2", "bag", "record",
        MODULE.RECORDER_INCLUDE_UNPUBLISHED_FLAG, "-s", "sqlite3", "-o",
        str(paths.bag.parent), MODULE.RECORDER_QOS_FLAG,
        authority_by_id["rosbag_qos_overrides"]["path"],
        MODULE.RECORDER_CACHE_FLAG, str(MODULE.RECORDER_CACHE_BYTES),
        MODULE.RECORDER_TOPICS_FLAG,
        *MODULE.RECORDER_TOPICS,
    ]
    flow_common = []
    for identifier, path_flag, sha_flag in (
        ("right_executor_source", "--right-executor",
         "--right-executor-sha256"),
        ("right_execution_authorization", "--right-manifest",
         "--right-manifest-sha256"),
        ("support_source", "--support-orchestrator",
         "--support-orchestrator-sha256"),
        ("middle_support_manifest", "--support-manifest",
         "--support-manifest-sha256"),
    ):
        flow_common += authority_flags(identifier, path_flag, sha_flag)
    prefix_argv = [
        "/usr/bin/python3", authority_by_id["flow_source"]["path"],
        "prefix", "--right-time-scale", "1.5", *flow_common,
    ]
    middle_argv = [
        "/usr/bin/python3", authority_by_id["flow_source"]["path"],
        "middle-and-suffix", "--right-time-scale", "1.5",
        "--middle-speed-scale", "0.5", *flow_common,
    ]
    for identifier, path_flag, sha_flag in (
        ("middle_plan", "--plan", "--plan-sha256"),
        ("middle_exact_result", "--result", "--result-sha256"),
        ("middle_certificate", "--certificate", "--certificate-sha256"),
        ("middle_executor_source", "--middle-executor",
         "--middle-executor-sha256"),
        ("middle_admission", "--middle-admission",
         "--middle-admission-sha256"),
        ("middle_execution_contract", "--middle-contract",
         "--middle-contract-sha256"),
    ):
        middle_argv += authority_flags(identifier, path_flag, sha_flag)
    postflight_argv = [
        "/usr/bin/python3", authority_by_id["support_source"]["path"],
        "finalize",
        *authority_flags("middle_support_manifest", "--manifest",
                         "--manifest-sha256"),
    ]
    argv = {
        "launch": launch_argv, "recorder": recorder_argv,
        "video": video_argv, "prefix": prefix_argv,
        "middle_suffix": middle_argv, "postflight": postflight_argv,
    }

    planned_replay_argv = [
        str(paths.replay_binary),
        "--scene", str(paths.scene_cdr),
        "--path", str(paths.contact_path),
        "--telemetry-tsv", str(paths.telemetry_tsv),
        "--urdf", str(paths.urdf),
        "--srdf", str(paths.srdf),
        "--handle-stl", str(paths.handle_stl),
    ]
    plan_value = {
        "schema": MODULE.PLAN_SCHEMA,
        "run_id": run_id,
        "sealed_monotonic_ns": 900_000_000,
        "input_paths": {
            "rosbag": str(paths.bag),
            "markers": str(paths.markers),
            "proc_snapshot": str(paths.proc),
            "video_audit": str(paths.video_audit),
            "postflight": str(paths.postflight),
            "workcell_pose_audit": str(paths.workcell),
            "gui_camera_audit": str(paths.camera),
            "video_window_audit": str(paths.window),
            "formal_authorities": str(paths.formal),
            "exact_fcl_replay": str(paths.replay),
        },
        "formal_authorities_sha256": formal_sha,
        "expected_process_argv_sha256": {
            role: _argv_sha(argv[role]) for role in MODULE.PROCESS_ROLES},
        "expected_exact_fcl_replay_argv_sha256": _argv_sha(
            planned_replay_argv),
        "expected_exact_fcl_file_sha256": {
            "auditor_source": replay_source_sha,
            "auditor_path_base_source": replay_path_base_source_sha,
            "auditor_middle_base_source": replay_middle_base_source_sha,
            "auditor_binary": replay_binary_sha,
            "contact_path": geometry_sha["path"],
            "original_handle_stl": geometry_sha["handle"],
            "scene_cdr": geometry_sha["scene"],
            "urdf": geometry_sha["urdf"],
            "srdf": geometry_sha["srdf"],
        },
        "evidence_tools": {
            "supervisor_source": bound(
                paths.supervisor_source, supervisor_sha, "0500"),
            "certificate_source": {
                "path": str(SOURCE), "sha256": certificate_sha,
                "size_bytes": SOURCE.stat().st_size, "mode": "0500",
            },
        },
        "max_joint_gap_ns": MODULE.MAX_JOINT_GAP_NS,
        "valid": True,
        "status": "READY",
    }
    plan_sha = _sealed(paths.plan, plan_value)

    marker_start = 1_100_000_000
    marker_bag_start = 1_700_000_000_000_000_000
    marker_step = 10_000_000
    marker_values = []
    for index, spec in enumerate(MODULE.MARKER_SPECS):
        # Every pattern has one deterministic representative in the fixture.
        line = {
            "CAPTURE_START": "H1_CANDIDATE3_LIVE_829=START",
            "CAPTURE_END": "H1_CANDIDATE3_LIVE_829=END",
        }.get(spec.name)
        if line is None:
            pattern = spec.pattern.pattern
            if "dense_points=" in pattern:
                line = pattern.replace("\\Z", "")
                line = line.replace("[1-9][0-9]*", "10")
                line = line.replace("\\_", "_").replace("\\-", "-")
                line = line.replace("\\=", "=").replace("\\ ", " ")
            else:
                # Literal patterns were built with re.escape.
                candidates = [
                    text for text in (
                        "H1_SUPPORT_ORCHESTRATOR_828=PASS "
                        "operation=preload door=detached cube=detached",
                        "RIGHT_HANDLE_EXECUTOR_828=PASS direction=prefix",
                        "CANDIDATE3_FLOW_828_PREFIX=PASS",
                        "RIGHT_HANDLE_EXECUTOR_828_Q90_E0=PASS "
                        "fresh_feedback=true controller_commands=0",
                        "H1_SUPPORT_ORCHESTRATOR_828=PASS "
                        "operation=door-attach door=attached cube=detached "
                        "boundary=E0_ENTRY",
                        "H1_SUPPORT_ORCHESTRATOR_828=PASS "
                        "operation=cube-attach door=attached cube=attached "
                        "boundary=CUBE_CONTACT",
                        "H1_SUPPORT_ORCHESTRATOR_828=PASS "
                        "operation=cube-detach door=attached cube=detached "
                        "boundary=CUBE_RELEASE",
                        "H1_SUPPORT_ORCHESTRATOR_828=PASS "
                        "operation=door-detach door=detached cube=detached "
                        "boundary=E0_RETURN",
                        "CANDIDATE3_MIDDLE_828=PASS phases=11 "
                        "right_gripper_commands=0 terminal_e0_held=true",
                        "RIGHT_HANDLE_EXECUTOR_828=PASS direction=suffix",
                        "RIGHT_HANDLE_EXECUTOR_828_LAUNCH_SEAM=PASS "
                        "fresh_feedback=true q0_released=true "
                        "reverse_ingress=true left_launch=true "
                        "torso_launch=true controller_commands=0",
                        "H1_SUPPORT_ORCHESTRATOR_828=PASS "
                        "operation=finalize door=q0 handle=q0 latch=locked "
                        "supports=detached robot=launch publishers=0",
                        "CANDIDATE3_FLOW_828_MIDDLE_SUFFIX=PASS",
                    ) if spec.pattern.fullmatch(text)]
                if not candidates and spec.name.startswith("MIDDLE_PHASE_"):
                    phase_index = int(spec.name.rsplit("_", 1)[1])
                    phase_names = (
                        "door_pre_support", "door_open_supported",
                        "door60_seam_to_cube", "left_cube_ready",
                        "cube_contact", "cube_carry_place", "cube_release",
                        "cube_retreat", "left_cube_park",
                        "door60_seam_to_hinge", "door_close_supported")
                    commands = (0, 2, 2, 1, 2, 2, 1, 1, 1, 2, 2)
                    candidates = [
                        "CANDIDATE3_MIDDLE_828_PHASE=PASS "
                        f"index={phase_index} name={phase_names[phase_index]} "
                        f"commands={commands[phase_index]}"]
                assert len(candidates) == 1, spec.name
                line = candidates[0]
        assert spec.pattern.fullmatch(line), (spec.name, line, spec.pattern)
        timestamp = marker_start + index * marker_step
        marker_values.append({
            "schema": MODULE.MARKER_SCHEMA,
            "run_id": run_id,
            "sequence": index,
            "monotonic_ns": timestamp,
            "bag_timestamp_ns": marker_bag_start + index * marker_step,
            "process_role": spec.process_role,
            "name": spec.name,
            "line": line,
        })
    paths.markers.write_bytes(b"".join(
        _canonical(value) for value in marker_values))
    paths.markers.chmod(0o400)
    left_gripper_plan_trajectories = (left_close, left_open)
    _create_bag(paths.bag, marker_values, tuple(
        _scheduled_middle_trajectory(value)
        for value in left_gripper_plan_trajectories))
    bag_sha = hashlib.sha256(paths.bag.read_bytes()).hexdigest()
    bag_evidence = MODULE._read_sealed_file(
        paths.bag, maximum_bytes=MODULE.MAX_DB_BYTES,
        status="TEST_ROSBAG")
    _, replay_telemetry = MODULE._analyze_rosbag(
        bag_evidence, marker_values, JsonCdrDecoder())
    telemetry_sha = _sealed(paths.telemetry_tsv, replay_telemetry.raw)
    stdout_raw, replay_counts = _fake_fcl_stdout(
        replay_telemetry, geometry_sha, telemetry_sha)
    replay_stdout_sha = _sealed(paths.replay_stdout, stdout_raw)
    replay_argv = list(planned_replay_argv)

    _sealed(paths.replay, {
        "schema": MODULE.EXACT_FCL_REPLAY_SCHEMA,
        "run_id": run_id,
        "scope": MODULE._exact_fcl_scope(),
        "rosbag": bound(paths.bag, bag_sha),
        "telemetry_tsv": bound(paths.telemetry_tsv, telemetry_sha),
        "geometry": {
            "contact_path": bound(paths.contact_path, geometry_sha["path"]),
            "original_handle_stl": bound(
                paths.handle_stl, geometry_sha["handle"]),
            "scene_cdr": bound(paths.scene_cdr, geometry_sha["scene"]),
            "urdf": bound(paths.urdf, geometry_sha["urdf"]),
            "srdf": bound(paths.srdf, geometry_sha["srdf"]),
        },
        "auditor": {
            "source": bound(paths.replay_source, replay_source_sha),
            "path_base_source": bound(
                paths.replay_path_base_source,
                replay_path_base_source_sha),
            "middle_base_source": bound(
                paths.replay_middle_base_source,
                replay_middle_base_source_sha),
            "binary": bound(
                paths.replay_binary, replay_binary_sha, "0500"),
            "argv": replay_argv,
            "argv_sha256": _argv_sha(replay_argv),
            "exit_code": 0,
            "stdout": bound(paths.replay_stdout, replay_stdout_sha),
            "stderr": bound(paths.replay_stderr, replay_stderr_sha),
        },
        "replay": {
            "source_joint_state_samples":
                replay_telemetry.source_sample_count,
            "densified_sample_count": len(replay_telemetry.rows),
            "maximum_source_gap_ns":
                replay_telemetry.maximum_source_gap_ns,
            "maximum_revolute_step_rad":
                replay_telemetry.maximum_revolute_step_rad,
            "maximum_linear_step_m":
                replay_telemetry.maximum_linear_step_m,
            "close_command_timestamp_ns":
                replay_telemetry.close_command_timestamp_ns,
            "close_pass_timestamp_ns":
                replay_telemetry.close_pass_timestamp_ns,
            "release_command_timestamp_ns":
                replay_telemetry.release_command_timestamp_ns,
            "release_pass_timestamp_ns":
                replay_telemetry.release_pass_timestamp_ns,
            "excluded_close_transition_rows":
                replay_telemetry.excluded_close_transition_rows,
            "excluded_release_transition_rows":
                replay_telemetry.excluded_release_transition_rows,
            "collision_free": True,
            "left_handle_checks": len(replay_telemetry.rows),
            "left_handle_contact_count": 0,
            "minimum_left_handle_clearance_m": 0.051,
            "right_contact_required_samples": replay_counts["contact"],
            "right_grip_engaged_samples": replay_counts["grip"],
            "right_pad1_patch_valid_samples": replay_counts["pad1"],
            "right_pad2_patch_valid_samples": replay_counts["pad2"],
            "maximum_contact_minimum_right_pad_distance_m": replay_counts[
                "maximum_contact_minimum_right_pad_distance_m"],
            "minimum_adjacent_finger_handle_clearance_m": replay_counts[
                "minimum_adjacent_finger_handle_clearance_m"],
            "minimum_buffered_nonpad_handle_clearance_m": replay_counts[
                "minimum_buffered_nonpad_handle_clearance_m"],
            "allowed_handle_contact_pairs": [
                list(pair) for pair in MODULE.ALLOWED_HANDLE_CONTACT_PAIRS],
            "observed_handle_contact_pairs": [
                list(pair) for pair in MODULE.ALLOWED_HANDLE_CONTACT_PAIRS],
            "unauthorized_handle_contact_pairs": [],
            "replay_transcript_sha256": replay_stdout_sha,
        },
        "native_physics_contacts": {
            "backend": "DART",
            "available": False,
            "reason": MODULE.NATIVE_CONTACT_UNAVAILABLE_REASON,
            "force_or_wrench_claimed": False,
        },
        "offline_replay": True,
        "live_telemetry_bound": True,
        "valid": True,
        "status": "PASS",
    })

    supervisor_pid = 8000
    marker_by_name = {item["name"]: item for item in marker_values}
    final_marker_time = marker_values[-1]["monotonic_ns"]
    intervals = {
        "launch": (1_000_000_000, final_marker_time + 20_000_000),
        "recorder": (1_020_000_000, final_marker_time + 20_000_000),
        "video": (1_080_000_000, final_marker_time + 20_000_000),
        "prefix": (
            marker_by_name["SUPPORT_PRELOAD"]["monotonic_ns"] - 1_000_000,
            marker_by_name["FLOW_PREFIX_PASS"]["monotonic_ns"] + 1_000_000),
        "middle_suffix": (
            marker_by_name["Q90_BEFORE_MIDDLE"]["monotonic_ns"] - 1_000_000,
            marker_by_name[
                "FLOW_MIDDLE_SUFFIX_PASS"]["monotonic_ns"] + 1_000_000),
        "postflight": (
            marker_by_name["POSTFLIGHT_FINALIZE"]["monotonic_ns"] -
            1_000_000,
            marker_by_name["POSTFLIGHT_FINALIZE"]["monotonic_ns"] +
            1_000_000),
    }
    processes = []
    for index, role in enumerate(MODULE.PROCESS_ROLES):
        processes.append({
            "role": role,
            "pid": 8100 + index,
            "ppid": supervisor_pid,
            "start_ticks": 10000 + index,
            "started_monotonic_ns": intervals[role][0],
            "ended_monotonic_ns": intervals[role][1],
            "exit_code": 0,
            "run_id": run_id,
            "argv": argv[role],
            "cmdline_sha256": _argv_sha(argv[role]),
        })
    _sealed(paths.proc, {
        "schema": MODULE.PROC_SCHEMA,
        "run_id": run_id,
        "boot_id_sha256": "b" * 64,
        "supervisor_pid": supervisor_pid,
        "supervisor_start_ticks": 9000,
        "sealed_plan_sha256": plan_sha,
        "preexisting_processes": [],
        "recorder_ready": {
            "captured_monotonic_ns": 1_060_000_000,
            "subscription_topics": sorted(MODULE.TOPIC_TYPES),
        },
        "captured_monotonic_ns": final_marker_time + 100_000_000,
        "processes": processes,
        "valid": True,
        "status": "PASS",
    })

    video_sha = _sealed(paths.video, b"synthetic h264 mp4 bytes")
    _sealed(paths.video_audit, {
        "schema": MODULE.VIDEO_SCHEMA,
        "run_id": run_id,
        "video": {
            "path": str(paths.video),
            "sha256": video_sha,
            "size_bytes": paths.video.stat().st_size,
            "mode": "0400",
        },
        "capture_started_monotonic_ns": 1_090_000_000,
        "capture_ended_monotonic_ns": final_marker_time + 10_000_000,
        "ffprobe": {
            "exit_code": 0,
            "stream_count": 1,
            "codec_name": "h264",
            "width": 1160,
            "height": 1530,
            "pix_fmt": "yuv420p",
            "avg_frame_rate_num": 12,
            "avg_frame_rate_den": 1,
            "duration_s": 0.5,
            "frame_count": 6,
        },
        "decode": {
            "exit_code": 0,
            "stderr_size_bytes": 0,
            "stderr_sha256": MODULE.EMPTY_SHA256,
        },
        "valid": True,
        "status": "PASS",
    })
    _sealed(paths.camera, {
        "schema": MODULE.GUI_CAMERA_SCHEMA,
        "run_id": run_id,
        "sealed_plan_sha256": plan_sha,
        "captured_monotonic_ns": 1_050_000_000,
        "source_command_argv": list(MODULE.GUI_CAMERA_ARGV),
        "source_command_argv_sha256": _argv_sha(MODULE.GUI_CAMERA_ARGV),
        "request": MODULE.GUI_CAMERA_REQUEST,
        "response_stdout": "data: true\n",
        "response_stderr": "",
        "exit_code": 0,
        "valid": True,
        "status": "PASS",
    })
    xwindow_id = 0x420001
    window_hex = hex(xwindow_id)
    tree_argv = ["/usr/bin/xwininfo", "-root", "-tree"]
    details_argv = ["/usr/bin/xwininfo", "-id", window_hex]
    ffmpeg_argv = [
        "/usr/bin/ffmpeg", "-n", "-thread_queue_size", "512", "-f",
        "x11grab", "-window_id", str(xwindow_id), "-draw_mouse", "0",
        "-framerate", "12", "-video_size", "2000x1690", "-i", ":1",
        "-an", "-vf", "crop=1160:1530:0:80", "-c:v", "libx264",
        "-preset", "ultrafast", "-crf", "20", "-pix_fmt", "yuv420p",
        str(paths.video),
    ]
    _sealed(paths.window, {
        "schema": MODULE.VIDEO_WINDOW_SCHEMA,
        "run_id": run_id,
        "sealed_plan_sha256": plan_sha,
        "helper_source_sha256": supervisor_sha,
        "helper_process_argv": video_argv,
        "helper_process_argv_sha256": _argv_sha(video_argv),
        "capture_started_monotonic_ns": 1_090_000_000,
        "xwindow_id_hex": window_hex,
        "xwindow_id": xwindow_id,
        "wm_class": ["gz-sim-gui", "Gazebo GUI"],
        "display": ":1",
        "window_width": 2000,
        "window_height": 1690,
        "width": 1160,
        "height": 1530,
        "crop": "1160:1530:0:80",
        "window_tree_argv": tree_argv,
        "window_tree_argv_sha256": _argv_sha(tree_argv),
        "window_details_argv": details_argv,
        "window_details_argv_sha256": _argv_sha(details_argv),
        "ffmpeg_argv": ffmpeg_argv,
        "ffmpeg_argv_sha256": _argv_sha(ffmpeg_argv),
        "valid": True,
        "status": "CAPTURE_STARTED",
    })

    initial_poses = {
        "h1_2": {
            "entity_id": 1, "position_m": [0.0, 0.0, 0.0],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
        "table": {
            "entity_id": 2, "position_m": [0.600, 0.0, 0.0],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
        "incubator": {
            "entity_id": 3, "position_m": [0.590, 0.0, 0.815],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0]},
        "green_cube": {
            "entity_id": 4, "position_m": [0.334, 0.377, 0.830],
            "orientation_xyzw": [
                0.0, 0.0, 0.7071067811865475,
                0.7071067811865476]},
    }
    terminal_poses = json.loads(json.dumps(initial_poses))
    terminal_poses["green_cube"]["position_m"] = [0.611, -0.030, 0.995]
    preflight_start = marker_by_name["CAPTURE_START"]["monotonic_ns"]
    postflight_start = marker_by_name[
        "POSTFLIGHT_FINALIZE"]["monotonic_ns"]
    terminal_target = {
        "position_m": [0.611, -0.030, 0.995],
        "orientation_xyzw": [
            0.0, 0.0, 0.7071067811865475,
            0.7071067811865476],
        "position_tolerance_m": 0.003,
        "orientation_tolerance_rad": 0.03490658503988659,
    }
    terminal_gates = [{
        "sequence": index,
        "position_error_m": 0.0,
        "orientation_error_rad": 0.0,
        "position_gate_passed": True,
        "orientation_gate_passed": True,
        "passed": True,
    } for index in range(3)]
    _sealed(paths.workcell, {
        "schema": MODULE.WORKCELL_POSE_SCHEMA,
        "run_id": run_id,
        "sealed_plan_sha256": plan_sha,
        "world_name": "h1_world",
        "source_command_argv": list(MODULE.WORKCELL_SOURCE_ARGV),
        "source_command_argv_sha256": _argv_sha(
            MODULE.WORKCELL_SOURCE_ARGV),
        "preflight_samples": [{
            "sequence": index,
            "monotonic_ns": preflight_start + 1_000_000 +
                index * 3_000_000,
            "entities": initial_poses,
        } for index in range(3)],
        "postflight_samples": [{
            "sequence": index,
            "monotonic_ns": postflight_start + index * 3_000_000,
            "entities": terminal_poses,
        } for index in range(3)],
        "terminal_cube_target": terminal_target,
        "terminal_cube_sample_gates": terminal_gates,
        "terminal_cube_gate_passed": True,
        "valid": True,
        "status": "PASS",
    })
    _sealed(paths.postflight, {
        "schema": MODULE.POSTFLIGHT_SCHEMA,
        "run_id": run_id,
        "captured_monotonic_ns": postflight_start + 5_000_000,
        "door_angle_rad": 0.001,
        "handle_angle_rad": -0.001,
        "torso_angle_rad": 0.0,
        "latch_locked": True,
        "supports_detached": True,
        "robot_at_launch": True,
        "controller_count": 6,
        "trajectory_publisher_count": 0,
        "unexpected_executor_process_count": 0,
        "clock_advancing": True,
        "valid": True,
        "status": "PASS",
    })
    return SimpleNamespace(
        root=root, paths=paths, plan_sha=plan_sha, run_id=run_id,
        marker_values=marker_values, authority_items=authority_items,
        authority_by_id=authority_by_id, decoder=JsonCdrDecoder(),
        left_gripper_plan_trajectories=left_gripper_plan_trajectories)


def _assemble(fixture):
    return MODULE.assemble(
        plan_path=fixture.paths.plan,
        plan_sha256=fixture.plan_sha,
        rosbag_path=fixture.paths.bag,
        markers_path=fixture.paths.markers,
        proc_snapshot_path=fixture.paths.proc,
        video_audit_path=fixture.paths.video_audit,
        postflight_path=fixture.paths.postflight,
        workcell_pose_audit_path=fixture.paths.workcell,
        gui_camera_audit_path=fixture.paths.camera,
        video_window_audit_path=fixture.paths.window,
        formal_authorities_path=fixture.paths.formal,
        exact_fcl_replay_path=fixture.paths.replay,
        output_path=fixture.paths.certificate,
        decoder=fixture.decoder)


def _wrap_replay(fixture):
    return MODULE.wrap_exact_fcl_replay(
        plan_path=fixture.paths.plan,
        plan_sha256=fixture.plan_sha,
        rosbag_path=fixture.paths.bag,
        markers_path=fixture.paths.markers,
        telemetry_tsv_path=fixture.paths.telemetry_tsv,
        scene_path=fixture.paths.scene_cdr,
        contact_path=fixture.paths.contact_path,
        urdf_path=fixture.paths.urdf,
        srdf_path=fixture.paths.srdf,
        handle_stl_path=fixture.paths.handle_stl,
        auditor_source_path=fixture.paths.replay_source,
        auditor_path_base_source_path=fixture.paths.replay_path_base_source,
        auditor_middle_base_source_path=
            fixture.paths.replay_middle_base_source,
        auditor_binary_path=fixture.paths.replay_binary,
        stdout_path=fixture.paths.replay_stdout,
        stderr_path=fixture.paths.replay_stderr,
        exit_code=0,
        output_path=fixture.paths.replay,
        decoder=fixture.decoder)


def _update_bag(path, statement, parameters=()):
    path.chmod(0o600)
    connection = sqlite3.connect(path)
    connection.execute(statement, parameters)
    connection.commit()
    connection.close()
    path.chmod(0o400)


def _rewrite_json(path, mutate):
    path.chmod(0o600)
    value = json.loads(path.read_text(encoding="utf-8"))
    mutate(value)
    path.write_bytes(_canonical(value))
    path.chmod(0o400)


def _set_machine_token(line, key, value):
    tokens = line.split(" ")
    matches = [index for index, token in enumerate(tokens)
               if token.startswith(f"{key}=")]
    assert len(matches) == 1
    tokens[matches[0]] = f"{key}={value}"
    return " ".join(tokens)


def _rewrite_fcl_stdout(fixture, mutate):
    fixture.paths.replay_stdout.chmod(0o600)
    lines = fixture.paths.replay_stdout.read_text(
        encoding="ascii").splitlines()
    mutate(lines)
    raw = ("\n".join(lines) + "\n").encode("ascii")
    fixture.paths.replay_stdout.write_bytes(raw)
    fixture.paths.replay_stdout.chmod(0o400)
    digest = hashlib.sha256(raw).hexdigest()

    def bind(value):
        binding = value["auditor"]["stdout"]
        binding["sha256"] = digest
        binding["size_bytes"] = len(raw)
        value["replay"]["replay_transcript_sha256"] = digest

    _rewrite_json(fixture.paths.replay, bind)


def _mutate_bag_payload(path, topic, mutate, *, offset=0):
    path.chmod(0o600)
    connection = sqlite3.connect(path)
    row = connection.execute(
        "SELECT m.id,m.data FROM messages m JOIN topics t "
        "ON t.id=m.topic_id WHERE t.name=? ORDER BY m.id LIMIT 1 OFFSET ?",
        (topic, offset)).fetchone()
    assert row is not None
    value = json.loads(bytes(row[1]).decode("utf-8"))
    mutate(value)
    connection.execute(
        "UPDATE messages SET data=? WHERE id=?",
        (sqlite3.Binary(_payload(value)), row[0]))
    connection.commit()
    connection.close()
    path.chmod(0o400)


def _shift_bag_header_stamps_from_offset(path, topic, offset, delta_ns):
    path.chmod(0o600)
    connection = sqlite3.connect(path)
    rows = connection.execute(
        "SELECT m.id,m.data FROM messages m JOIN topics t "
        "ON t.id=m.topic_id WHERE t.name=? ORDER BY m.id",
        (topic,)).fetchall()
    assert 0 < offset < len(rows)
    for message_id, raw in rows[offset:]:
        value = json.loads(bytes(raw).decode("utf-8"))
        value["header_stamp_ns"] += delta_ns
        connection.execute(
            "UPDATE messages SET data=? WHERE id=?",
            (sqlite3.Binary(_payload(value)), message_id))
    connection.commit()
    connection.close()
    path.chmod(0o400)


def _analyze_fixture_bag(fixture):
    bag = MODULE._read_sealed_file(
        fixture.paths.bag, maximum_bytes=MODULE.MAX_DB_BYTES,
        status="TEST_ROSBAG")
    return MODULE._analyze_rosbag(
        bag, fixture.marker_values, fixture.decoder)


def _mutated_formal_summary(fixture, identifier, mutate):
    items = json.loads(json.dumps(fixture.authority_items))
    item = next(value for value in items if value["id"] == identifier)
    path = Path(item["path"])
    old_mode = path.stat().st_mode & 0o777
    path.chmod(0o600)
    value = json.loads(path.read_text(encoding="utf-8"))
    mutate(value)
    path.write_bytes(_canonical(value))
    path.chmod(old_mode)
    item["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    item["size_bytes"] = path.stat().st_size
    return {"authority_bindings": items}


def test_assemble_and_independent_verify_are_ros_free(tmp_path):
    fixture = _fixture(tmp_path)
    assert MODULE.FORMAL_AUTHORITY_IDS[-5:] == (
        "robot_gazebo_control_xacro", "ros2_controllers_config",
        "robot_joint_state_relay_source", "telemetry_bridge_config",
        "rosbag_qos_overrides",
    )
    artifact = _assemble(fixture)
    assert artifact.path == fixture.paths.certificate
    assert artifact.value["status"] == "PASS"
    assert artifact.value["valid"] is True
    assert artifact.value["live_execution_passed"] is True
    assert artifact.value["telemetry_and_commands"][
        "controller_message_counts"] == dict(sorted(
            MODULE.CONTROLLER_COUNTS.items()))
    assert artifact.value["telemetry_and_commands"][
        "middle_right_gripper_messages"] == 0
    assert artifact.value["telemetry_and_commands"][
        "maximum_abs_torso_rad"] == pytest.approx(0.5)
    assert artifact.value["telemetry_and_commands"][
        "maximum_leg_joint_drift_rad"] > 0.025
    assert artifact.value["dynamic_cube_delegation"][
        "terminal_cube_target"]["position_m"] == [0.611, -0.030, 0.995]
    assert artifact.value["workcell_pose"][
        "terminal_cube_gate_passed"] is True
    assert artifact.value["evidence_tools"]["certificate_source"][
        "path"] == str(SOURCE)
    assert fixture.paths.certificate.stat().st_mode & 0o777 == 0o400
    verified = MODULE.verify(
        certificate_path=fixture.paths.certificate,
        certificate_sha256=artifact.sha256,
        plan_sha256=fixture.plan_sha,
        decoder=fixture.decoder)
    assert verified == artifact.value


def test_proc_rejects_offline_manifest_as_controller_authorization(tmp_path):
    fixture = _fixture(tmp_path)
    proc = json.loads(fixture.paths.proc.read_text(encoding="utf-8"))
    plan = json.loads(fixture.paths.plan.read_text(encoding="utf-8"))
    authorities = {item["id"]: item for item in fixture.authority_items}
    offline_manifest = authorities["right_manifest"]
    for role in ("prefix", "middle_suffix"):
        process = next(item for item in proc["processes"]
                       if item["role"] == role)
        argv = process["argv"]
        argv[argv.index("--right-manifest") + 1] = offline_manifest["path"]
        argv[argv.index("--right-manifest-sha256") + 1] = (
            offline_manifest["sha256"])
        process["cmdline_sha256"] = _argv_sha(argv)
        plan["expected_process_argv_sha256"][role] = _argv_sha(argv)
    with pytest.raises(
            MODULE.LiveCertificateError,
            match="PROC_FLOW_AUTHORITY:right_execution_authorization"):
        MODULE._validate_proc_snapshot(
            proc, plan, fixture.plan_sha, fixture.paths.plan,
            fixture.marker_values,
            {"authority_bindings": fixture.authority_items})


@pytest.mark.parametrize(("identifier", "path_flag", "sha_flag"), (
    ("robot_gazebo_control_xacro", "--robot-gazebo-control-xacro",
     "--robot-gazebo-control-xacro-sha256"),
    ("ros2_controllers_config", "--ros2-controllers-config",
     "--ros2-controllers-config-sha256"),
    ("robot_joint_state_relay_source", "--robot-joint-state-relay-source",
     "--robot-joint-state-relay-source-sha256"),
    ("telemetry_bridge_config", "--telemetry-bridge-config",
     "--telemetry-bridge-config-sha256"),
))
def test_proc_rejects_new_launch_authority_tamper(
        tmp_path, identifier, path_flag, sha_flag):
    fixture = _fixture(tmp_path)
    proc = json.loads(fixture.paths.proc.read_text(encoding="utf-8"))
    plan = json.loads(fixture.paths.plan.read_text(encoding="utf-8"))
    launch = next(item for item in proc["processes"]
                  if item["role"] == "launch")
    replacement = fixture.authority_by_id["middle_admission"]
    launch["argv"][launch["argv"].index(path_flag) + 1] = replacement["path"]
    launch["argv"][launch["argv"].index(sha_flag) + 1] = replacement["sha256"]
    launch["cmdline_sha256"] = _argv_sha(launch["argv"])
    plan["expected_process_argv_sha256"]["launch"] = launch[
        "cmdline_sha256"]
    with pytest.raises(
            MODULE.LiveCertificateError,
            match=f"PROC_LAUNCH_AUTHORITY:{identifier}"):
        MODULE._validate_proc_snapshot(
            proc, plan, fixture.plan_sha, fixture.paths.plan,
            fixture.marker_values,
            {"authority_bindings": fixture.authority_items})


@pytest.mark.parametrize("assignment", (
    "robot_joint_state_relay_source", "robot_joint_state_relay_sha256",
    "telemetry_bridge_config", "telemetry_bridge_config_sha256",
))
def test_proc_rejects_telemetry_launch_assignment_tamper(
        tmp_path, assignment):
    fixture = _fixture(tmp_path)
    proc = json.loads(fixture.paths.proc.read_text(encoding="utf-8"))
    plan = json.loads(fixture.paths.plan.read_text(encoding="utf-8"))
    launch = next(item for item in proc["processes"]
                  if item["role"] == "launch")
    prefix = assignment + ":="
    index = next(index for index, item in enumerate(launch["argv"])
                 if item.startswith(prefix))
    launch["argv"][index] = prefix + ("f" * 64)
    launch["cmdline_sha256"] = _argv_sha(launch["argv"])
    plan["expected_process_argv_sha256"]["launch"] = launch[
        "cmdline_sha256"]
    with pytest.raises(
            MODULE.LiveCertificateError,
            match="PROC_LAUNCH_FIXED_ASSIGNMENTS"):
        MODULE._validate_proc_snapshot(
            proc, plan, fixture.plan_sha, fixture.paths.plan,
            fixture.marker_values,
            {"authority_bindings": fixture.authority_items})


def test_proc_rejects_rosbag_qos_authority_tamper(tmp_path):
    fixture = _fixture(tmp_path)
    proc = json.loads(fixture.paths.proc.read_text(encoding="utf-8"))
    plan = json.loads(fixture.paths.plan.read_text(encoding="utf-8"))
    recorder = next(item for item in proc["processes"]
                    if item["role"] == "recorder")
    replacement = fixture.authority_by_id["middle_admission"]
    qos_index = recorder["argv"].index(MODULE.RECORDER_QOS_FLAG) + 1
    recorder["argv"][qos_index] = replacement["path"]
    recorder["cmdline_sha256"] = _argv_sha(recorder["argv"])
    plan["expected_process_argv_sha256"]["recorder"] = recorder[
        "cmdline_sha256"]
    with pytest.raises(
            MODULE.LiveCertificateError,
            match="PROC_RECORDER_QOS_CACHE_CONTRACT"):
        MODULE._validate_proc_snapshot(
            proc, plan, fixture.plan_sha, fixture.paths.plan,
            fixture.marker_values,
            {"authority_bindings": fixture.authority_items})


def test_public_extract_and_wrap_stages_reproduce_sealed_artifacts(tmp_path):
    fixture = _fixture(tmp_path)
    expected_tsv = fixture.paths.telemetry_tsv.read_bytes()
    expected_replay = fixture.paths.replay.read_bytes()
    fixture.paths.telemetry_tsv.unlink()
    fixture.paths.replay.unlink()

    extracted = MODULE.extract_fcl_telemetry(
        plan_path=fixture.paths.plan,
        plan_sha256=fixture.plan_sha,
        rosbag_path=fixture.paths.bag,
        markers_path=fixture.paths.markers,
        output_path=fixture.paths.telemetry_tsv,
        decoder=fixture.decoder)
    assert extracted.raw == expected_tsv
    assert fixture.paths.telemetry_tsv.stat().st_mode & 0o777 == 0o400

    wrapped = _wrap_replay(fixture)
    assert wrapped.raw == expected_replay
    assert wrapped.value["scope"] == MODULE._exact_fcl_scope()
    assert wrapped.value["replay"][
        "excluded_close_transition_rows"] > 0
    assert wrapped.value["replay"][
        "excluded_release_transition_rows"] > 0
    artifact = _assemble(fixture)
    assert artifact.value["contact_evidence"]["scope"][
        "dynamic_cube_objects_included"] is False


def test_wrap_rejects_tsv_not_derived_from_bound_db3(tmp_path):
    fixture = _fixture(tmp_path)
    fixture.paths.telemetry_tsv.chmod(0o600)
    fixture.paths.telemetry_tsv.write_bytes(
        fixture.paths.telemetry_tsv.read_bytes() + b"tampered\n")
    fixture.paths.telemetry_tsv.chmod(0o400)
    with pytest.raises(
            MODULE.LiveCertificateError,
            match="EXACT_FCL_REPLAY_TELEMETRY_NOT_DERIVED_FROM_DB3"):
        _wrap_replay(fixture)


def test_exact_controller_counts_are_recomputed_from_sqlite(tmp_path):
    fixture = _fixture(tmp_path)
    _update_bag(
        fixture.paths.bag,
        "DELETE FROM messages WHERE id=(SELECT m.id FROM messages m "
        "JOIN topics t ON t.id=m.topic_id WHERE t.name=? LIMIT 1)",
        ("/torso_controller/joint_trajectory",))
    with pytest.raises(
            MODULE.LiveCertificateError,
            match="ROSBAG_CONTROLLER_MESSAGE_COUNTS"):
        _assemble(fixture)
    assert not fixture.paths.certificate.exists()


def test_middle_window_rejects_right_gripper_even_with_exact_total(tmp_path):
    fixture = _fixture(tmp_path)
    middle_timestamp = next(
        item["bag_timestamp_ns"] for item in fixture.marker_values
        if item["name"] == "MIDDLE_PHASE_5")
    _update_bag(
        fixture.paths.bag,
        "UPDATE messages SET timestamp=? WHERE id=(SELECT m.id FROM messages m "
        "JOIN topics t ON t.id=m.topic_id WHERE t.name=? ORDER BY m.id LIMIT 1)",
        (middle_timestamp, "/right_gripper_controller/joint_trajectory"))
    with pytest.raises(
            MODULE.LiveCertificateError,
            match="ROSBAG_MIDDLE_RIGHT_GRIPPER_NONZERO"):
        _assemble(fixture)


def test_torso_limit_is_checked_in_continuous_joint_telemetry(tmp_path):
    fixture = _fixture(tmp_path)
    bad_state = _payload({
        "header_stamp_ns": 100_000_000_000,
        "names": list(MODULE.ALL_ROBOT_JOINTS),
        "positions": [
            math.radians(44.5001) if name == "torso_joint" else 0.0
            for name in MODULE.ALL_ROBOT_JOINTS],
    })
    _update_bag(
        fixture.paths.bag,
        "UPDATE messages SET data=? WHERE id=(SELECT m.id FROM messages m "
        "JOIN topics t ON t.id=m.topic_id WHERE t.name=? ORDER BY m.id LIMIT 1)",
        (sqlite3.Binary(bad_state), "/joint_states"))
    with pytest.raises(
            MODULE.LiveCertificateError,
            match="TORSO_LIMIT_44_5_DEG"):
        _assemble(fixture)


def test_marker_order_and_support_lines_are_fail_closed(tmp_path):
    fixture = _fixture(tmp_path)
    values = list(fixture.marker_values)
    values[1] = dict(values[1], name="SUPPORT_DOOR_ATTACH")
    fixture.paths.markers.chmod(0o600)
    fixture.paths.markers.write_bytes(b"".join(
        _canonical(value) for value in values))
    fixture.paths.markers.chmod(0o400)
    with pytest.raises(MODULE.LiveCertificateError, match="MARKER_1_ORDER"):
        _assemble(fixture)


def test_authority_symlink_and_existing_output_are_rejected(tmp_path):
    fixture = _fixture(tmp_path)
    first = Path(fixture.authority_items[0]["path"])
    backup = first.with_name("authority_backup.bin")
    backup.write_bytes(first.read_bytes())
    backup.chmod(0o400)
    first.unlink()
    first.symlink_to(backup)
    with pytest.raises(
            MODULE.LiveCertificateError,
            match="FORMAL_AUTHORITY_right_execution_authorization_PATH_ALIAS"):
        _assemble(fixture)

    fixture = _fixture(tmp_path / "second")
    artifact = _assemble(fixture)
    original = fixture.paths.certificate.read_bytes()
    with pytest.raises(MODULE.LiveCertificateError, match="OUTPUT_EXISTS"):
        _assemble(fixture)
    assert fixture.paths.certificate.read_bytes() == original
    assert hashlib.sha256(original).hexdigest() == artifact.sha256


def test_verify_detects_post_assembly_evidence_mutation(tmp_path):
    fixture = _fixture(tmp_path)
    artifact = _assemble(fixture)
    _rewrite_json(
        fixture.paths.postflight,
        lambda value: value.__setitem__("torso_angle_rad", 0.1))
    with pytest.raises(
            MODULE.LiveCertificateError,
            match="VERIFY_INPUT_postflight_(SHA256|SIZE_MISMATCH)"):
        MODULE.verify(
            certificate_path=fixture.paths.certificate,
            certificate_sha256=artifact.sha256,
            plan_sha256=fixture.plan_sha,
            decoder=fixture.decoder)


def test_nonfinite_json_and_video_coverage_are_rejected(tmp_path):
    fixture = _fixture(tmp_path)
    fixture.paths.postflight.chmod(0o600)
    raw = fixture.paths.postflight.read_text(encoding="utf-8")
    raw = raw.replace('"torso_angle_rad":0.0', '"torso_angle_rad":NaN')
    fixture.paths.postflight.write_text(raw, encoding="utf-8")
    fixture.paths.postflight.chmod(0o400)
    with pytest.raises(MODULE.LiveCertificateError, match="NONFINITE_JSON"):
        _assemble(fixture)

    fixture = _fixture(tmp_path / "video")
    _rewrite_json(
        fixture.paths.video_audit,
        lambda value: value.__setitem__(
            "capture_started_monotonic_ns", 1_110_000_000))
    with pytest.raises(MODULE.LiveCertificateError, match="VIDEO_FULL_COVERAGE"):
        _assemble(fixture)


@pytest.mark.parametrize(("section", "key", "replacement", "failure"), (
    ("scope", "dynamic_cube_objects_included", True,
     "EXACT_FCL_REPLAY_SCOPE"),
    ("replay", "left_handle_contact_count", 1,
     "EXACT_FCL_REPLAY_LEFT_HANDLE_CONTACT"),
    ("replay", "excluded_close_transition_rows", 0,
     "EXACT_FCL_REPLAY_TRANSITION_WINDOW_BINDING"),
    ("replay", "right_grip_engaged_samples", 699,
     "EXACT_FCL_REPLAY_RIGHT_GRIP_CONTRACT"),
    ("replay", "maximum_contact_minimum_right_pad_distance_m", 0.00031,
     "EXACT_FCL_REPLAY_RIGHT_GRIP_CONTRACT"),
    ("replay", "minimum_adjacent_finger_handle_clearance_m", -1e-12,
     "EXACT_FCL_REPLAY_RIGHT_GRIP_CONTRACT"),
    ("replay", "minimum_buffered_nonpad_handle_clearance_m", 0.00059,
     "EXACT_FCL_REPLAY_RIGHT_GRIP_CONTRACT"),
    ("replay", "right_pad1_patch_valid_samples", 999999,
     "EXACT_FCL_REPLAY_METRIC_BINDING"),
    ("native_physics_contacts", "force_or_wrench_claimed", True,
     "EXACT_FCL_REPLAY_NATIVE_CONTACTS_UNAVAILABLE"),
))
def test_exact_fcl_replay_contact_claims_are_fail_closed(
        tmp_path, section, key, replacement, failure):
    fixture = _fixture(tmp_path)
    _rewrite_json(
        fixture.paths.replay,
        lambda value: value[section].__setitem__(key, replacement))
    with pytest.raises(MODULE.LiveCertificateError, match=failure):
        _assemble(fixture)


def test_exact_fcl_right_grip_accepts_pad2_only_rows(tmp_path):
    fixture = _fixture(tmp_path)
    contact_lines = [
        line for line in fixture.paths.replay_stdout.read_text(
            encoding="ascii").splitlines()[:-1]
        if " contact_required=true " in f" {line} "
    ]
    distances = [dict(token.split("=", 1) for token in line.split(" ")[1:])
                 for line in contact_lines]
    assert any(
        float(row["pad1_handle_distance_m"]) >
        MODULE.RIGHT_PAD_GRIP_PROXIMITY_M and
        float(row["pad2_handle_distance_m"]) <=
        MODULE.RIGHT_PAD_GRIP_PROXIMITY_M
        for row in distances)
    artifact = _assemble(fixture)
    evidence = artifact.value["contact_evidence"]
    assert evidence["right_grip_engaged_samples"] == evidence[
        "right_contact_required_samples"]
    assert evidence["right_pad1_patch_valid_samples"] < evidence[
        "right_contact_required_samples"]
    assert evidence[
        "maximum_contact_minimum_right_pad_distance_m"] == pytest.approx(
            0.0002)


def test_exact_fcl_observed_pairs_come_from_contacts_not_patch_diagnostics(
        tmp_path):
    fixture = _fixture(tmp_path)

    def remove_pad1_patch_diagnostics(lines):
        for index in range(len(lines) - 1):
            lines[index] = _set_machine_token(
                lines[index], "pad1_patch_valid", "false")
        lines[-1] = _set_machine_token(
            lines[-1], "pad1_patch_valid", "0")

    _rewrite_fcl_stdout(fixture, remove_pad1_patch_diagnostics)
    _rewrite_json(
        fixture.paths.replay,
        lambda value: value["replay"].__setitem__(
            "right_pad1_patch_valid_samples", 0))
    artifact = _assemble(fixture)
    assert artifact.value["contact_evidence"][
        "observed_handle_contact_pairs"] == [
            list(pair) for pair in MODULE.ALLOWED_HANDLE_CONTACT_PAIRS]


@pytest.mark.parametrize(("contact_required", "changes", "failure"), (
    (True, {
        "right_pad_grip_engaged": "false",
        "pad1_handle_distance_m": "0.00031",
        "pad2_handle_distance_m": "0.00031",
    }, "EXACT_FCL_REPLAY_ROW_[0-9]+_RIGHT_GRIP_REQUIRED"),
    (False, {
        "right_pad_grip_engaged": "true",
        "pad2_handle_distance_m": "0.0003",
    }, "EXACT_FCL_REPLAY_ROW_[0-9]+_RIGHT_GRIP_FORBIDDEN"),
    (False, {
        "pad1_contacts": "1",
    }, "EXACT_FCL_REPLAY_ROW_[0-9]+_NONCONTACT_PAD_CONTACT"),
    (True, {
        "right_pad_grip_engaged": "false",
    }, "EXACT_FCL_REPLAY_ROW_[0-9]+_GRIP_DISTANCE_BINDING"),
    (True, {
        "adjacent_finger_handle_clearance_m": "-0.000000000001",
    }, "EXACT_FCL_REPLAY_ROW_[0-9]+_CLEARANCE"),
    (True, {
        "nonpad_handle_clearance_m": "0.000599999999",
    }, "EXACT_FCL_REPLAY_ROW_[0-9]+_CLEARANCE"),
))
def test_exact_fcl_row_grip_geometry_is_fail_closed(
        tmp_path, contact_required, changes, failure):
    fixture = _fixture(tmp_path)

    def mutate(lines):
        token = f"contact_required={str(contact_required).lower()}"
        index = next(
            offset for offset, line in enumerate(lines[:-1])
            if token in line.split(" "))
        for key, value in changes.items():
            lines[index] = _set_machine_token(lines[index], key, value)

    _rewrite_fcl_stdout(fixture, mutate)
    with pytest.raises(MODULE.LiveCertificateError, match=failure):
        _assemble(fixture)


@pytest.mark.parametrize(("key", "replacement", "failure"), (
    ("right_pad_grip_engaged", "0",
     "EXACT_FCL_REPLAY_SUMMARY_COUNT:right_pad_grip_engaged"),
    ("max_contact_minimum_right_pad_distance_m", "0.00019",
     "EXACT_FCL_REPLAY_SUMMARY_METRIC:"
     "max_contact_minimum_right_pad_distance_m"),
    ("min_adjacent_finger_handle_clearance_m", "0.000009",
     "EXACT_FCL_REPLAY_SUMMARY_METRIC:"
     "min_adjacent_finger_handle_clearance_m"),
))
def test_exact_fcl_summary_grip_geometry_is_row_bound(
        tmp_path, key, replacement, failure):
    fixture = _fixture(tmp_path)
    _rewrite_fcl_stdout(
        fixture,
        lambda lines: lines.__setitem__(
            -1, _set_machine_token(lines[-1], key, replacement)))
    with pytest.raises(MODULE.LiveCertificateError, match=failure):
        _assemble(fixture)


def test_exact_fcl_replay_is_bound_to_the_same_db3_bytes(tmp_path):
    fixture = _fixture(tmp_path)
    _rewrite_json(
        fixture.paths.replay,
        lambda value: value["rosbag"].__setitem__("sha256", "e" * 64))
    with pytest.raises(
            MODULE.LiveCertificateError,
            match="EXACT_FCL_REPLAY_ROSBAG_SHA256"):
        _assemble(fixture)


def test_marker_jsonl_is_independently_rebound_to_bag_payload(tmp_path):
    fixture = _fixture(tmp_path)

    def tamper(value):
        payload = json.loads(value["data"])
        payload["name"] = "CAPTURE_END"
        value["data"] = _canonical(payload)[:-1].decode("ascii")

    _mutate_bag_payload(fixture.paths.bag, MODULE.MARKER_TOPIC, tamper)
    with pytest.raises(MODULE.LiveCertificateError, match="ROSBAG_MARKER_0_BINDING"):
        _assemble(fixture)


def test_joint_gap_uses_header_sim_time_not_sqlite_storage_time(tmp_path):
    fixture = _fixture(tmp_path)
    _update_bag(
        fixture.paths.bag,
        "UPDATE messages SET timestamp=timestamp+60000000 "
        "WHERE topic_id=(SELECT id FROM topics WHERE name=?) AND "
        "id>=(SELECT m.id FROM messages m JOIN topics t ON t.id=m.topic_id "
        "WHERE t.name=? ORDER BY m.id LIMIT 1 OFFSET 5)",
        ("/joint_states", "/joint_states"))
    summary, _ = _analyze_fixture_bag(fixture)
    assert summary["maximum_joint_state_gap_ns"] == 10_000_000

    fixture = _fixture(tmp_path / "sim_gap")
    _mutate_bag_payload(
        fixture.paths.bag, "/joint_states",
        lambda value: value.__setitem__(
            "header_stamp_ns", value["header_stamp_ns"] + 60_000_000),
        offset=5)
    with pytest.raises(
            MODULE.LiveCertificateError,
            match="ROSBAG_JOINT_STATE_SIM_TIME_GAP"):
        _analyze_fixture_bag(fixture)


@pytest.mark.parametrize(("delta_ns", "gap_ns"), (
    (-1_000_000, 9_000_000),
    (+1_000_000, 11_000_000),
), ids=("nine_ms", "eleven_ms"))
def test_robot_joint_state_requires_every_gap_to_be_exactly_ten_ms(
        tmp_path, delta_ns, gap_ns):
    fixture = _fixture(tmp_path)
    _shift_bag_header_stamps_from_offset(
        fixture.paths.bag, "/joint_states", 1, delta_ns)
    with pytest.raises(
            MODULE.LiveCertificateError,
            match="ROSBAG_JOINT_STATE_SIM_TIME_GAP"):
        _analyze_fixture_bag(fixture)

    bag = MODULE._read_sealed_file(
        fixture.paths.bag, maximum_bytes=MODULE.MAX_DB_BYTES,
        status="TEST_ROSBAG_GAP")
    connection = sqlite3.connect(
        f"file:{bag.path}?mode=ro&immutable=1", uri=True)
    topic_id = connection.execute(
        "SELECT id FROM topics WHERE name='/joint_states'").fetchone()[0]
    payloads = connection.execute(
        "SELECT data FROM messages WHERE topic_id=? ORDER BY id LIMIT 2",
        (topic_id,)).fetchall()
    connection.close()
    stamps = [json.loads(bytes(row[0]).decode("utf-8"))["header_stamp_ns"]
              for row in payloads]
    assert stamps[1] - stamps[0] == gap_ns


@pytest.mark.parametrize(("topic", "field", "failure"), (
    ("/clock", "clock_ns", "ROSBAG_CLOCK_ORDER"),
    ("/incubator/joint_states", "header_stamp_ns",
     "ROSBAG_INCUBATOR_STATE_SIM_TIME_GAP"),
))
def test_native_clock_and_incubator_require_every_gap_to_be_exactly_one_ms(
        tmp_path, topic, field, failure):
    fixture = _fixture(tmp_path)
    _mutate_bag_payload(
        fixture.paths.bag, topic,
        lambda value: value.__setitem__(
            field, value[field] + MODULE.NATIVE_SIM_PERIOD_NS),
        offset=5)
    with pytest.raises(MODULE.LiveCertificateError, match=failure):
        _analyze_fixture_bag(fixture)


def test_door_range_accepts_exact_fifty_microradian_tolerance(tmp_path):
    fixture = _fixture(tmp_path)
    _mutate_bag_payload(
        fixture.paths.bag, "/incubator/joint_states",
        lambda value: value["positions"].__setitem__(
            0, -MODULE.DOOR_PHYSICS_RANGE_TOLERANCE_RAD))
    summary, _ = _analyze_fixture_bag(fixture)
    assert summary["maximum_joint_state_gap_ns"] == 10_000_000


def test_door_range_rejects_value_beyond_fifty_microradians(tmp_path):
    fixture = _fixture(tmp_path)
    _mutate_bag_payload(
        fixture.paths.bag, "/incubator/joint_states",
        lambda value: value["positions"].__setitem__(
            0, -(MODULE.DOOR_PHYSICS_RANGE_TOLERANCE_RAD + 1e-12)))
    with pytest.raises(
            MODULE.LiveCertificateError, match="REPLAY_TSV_DOOR_RANGE"):
        _analyze_fixture_bag(fixture)


@pytest.mark.parametrize("mutation", ("missing", "extra", "duplicate", "shuffle"))
def test_joint_state_requires_exact_ordered_31_joint_vector(
        tmp_path, mutation):
    fixture = _fixture(tmp_path)

    def mutate(value):
        if mutation == "missing":
            value["names"].pop()
            value["positions"].pop()
        elif mutation == "extra":
            value["names"].append("invented_joint")
            value["positions"].append(0.0)
        elif mutation == "duplicate":
            value["names"][-1] = value["names"][0]
        else:
            value["names"][0], value["names"][1] = (
                value["names"][1], value["names"][0])
            value["positions"][0], value["positions"][1] = (
                value["positions"][1], value["positions"][0])

    _mutate_bag_payload(fixture.paths.bag, "/joint_states", mutate)
    with pytest.raises(MODULE.LiveCertificateError, match="JOINT"):
        _analyze_fixture_bag(fixture)


def test_leg_vector_is_in_tsv_and_tsv_tamper_is_rejected(tmp_path):
    fixture = _fixture(tmp_path)
    lines = fixture.paths.telemetry_tsv.read_text(encoding="ascii").splitlines()
    assert lines[0] == MODULE.REPLAY_TSV_HEADER
    fields = lines[1].split("\t")
    assert len(fields) == 10
    assert len(fields[5].split(",")) == 12
    fixture.paths.telemetry_tsv.chmod(0o600)
    leg_values = fields[5].split(",")
    leg_values[0] = "0.2"
    fields[5] = ",".join(leg_values)
    lines[1] = "\t".join(fields)
    fixture.paths.telemetry_tsv.write_text(
        "\n".join(lines) + "\n", encoding="ascii")
    fixture.paths.telemetry_tsv.chmod(0o400)
    with pytest.raises(
            MODULE.LiveCertificateError,
            match="EXACT_FCL_REPLAY_TELEMETRY_NOT_DERIVED_FROM_DB3"):
        _wrap_replay(fixture)


def test_delayed_middle_markers_use_sealed_left_gripper_payload(tmp_path):
    fixture = _fixture(tmp_path)
    artifact = _assemble(fixture)
    left = artifact.value["telemetry_and_commands"][
        "gripper_stage_topology"][-1]
    phase_zero = next(
        marker["bag_timestamp_ns"] for marker in fixture.marker_values
        if marker["name"] == "MIDDLE_PHASE_0")
    assert max(left["command_storage_timestamps_ns"]) < phase_zero
    assert left["phase_attribution"] == "sealed_middle_plan_payload"

    fixture = _fixture(tmp_path / "payload")
    _mutate_bag_payload(
        fixture.paths.bag,
        "/left_gripper_controller/joint_trajectory",
        lambda value: value["points"][0]["positions"].__setitem__(0, 0.012))
    with pytest.raises(
            MODULE.LiveCertificateError,
            match="DELEGATION_LEFT_GRIPPER_LIVE_PAYLOAD_MISMATCH"):
        _assemble(fixture)


@pytest.mark.parametrize("mutation", (
    "wrong_speed_scale", "wrong_fixed_lead", "single_point_timestamp",
))
def test_left_gripper_live_payload_rejects_execution_timestamp_tamper(
        tmp_path, mutation):
    fixture = _fixture(tmp_path)

    def tamper(value):
        if mutation == "wrong_speed_scale":
            wrong = _scheduled_middle_trajectory(
                fixture.left_gripper_plan_trajectories[0],
                speed_scale=0.75)
            value["points"] = wrong["points"]
        elif mutation == "wrong_fixed_lead":
            for point in value["points"]:
                point["time_from_start_ns"] += 1
        else:
            value["points"][1]["time_from_start_ns"] += 1

    _mutate_bag_payload(
        fixture.paths.bag,
        "/left_gripper_controller/joint_trajectory", tamper)
    with pytest.raises(
            MODULE.LiveCertificateError,
            match="DELEGATION_LEFT_GRIPPER_LIVE_PAYLOAD_MISMATCH"):
        _assemble(fixture)


def test_recorder_ready_subscription_barrier_is_fail_closed(tmp_path):
    fixture = _fixture(tmp_path)
    _rewrite_json(
        fixture.paths.proc,
        lambda value: value["recorder_ready"][
            "subscription_topics"].remove(MODULE.MARKER_TOPIC))
    with pytest.raises(
            MODULE.LiveCertificateError,
            match="PROC_RECORDER_READY_BINDING"):
        _assemble(fixture)


@pytest.mark.parametrize("flag_count", (0, 2))
def test_recorder_command_requires_exactly_one_include_unpublished_flag(
        flag_count):
    qos_binding = {
        "path": "/tmp/candidate3_qos.yaml", "mode": "0400",
        "sha256": MODULE.RECORDER_QOS_OVERRIDES_SHA256,
    }
    argv = [
        "/opt/ros/jazzy/bin/ros2", "bag", "record",
        *([MODULE.RECORDER_INCLUDE_UNPUBLISHED_FLAG] * flag_count),
        "-s", "sqlite3", "-o", "/tmp/candidate3_bag",
        MODULE.RECORDER_QOS_FLAG, qos_binding["path"],
        MODULE.RECORDER_CACHE_FLAG, str(MODULE.RECORDER_CACHE_BYTES),
        MODULE.RECORDER_TOPICS_FLAG, *MODULE.RECORDER_TOPICS,
    ]
    with pytest.raises(
            MODULE.LiveCertificateError,
            match="PROC_RECORDER_INCLUDE_UNPUBLISHED_TOPICS"):
        MODULE._validate_recorder_argv(
            argv, "/tmp/candidate3_bag", MODULE.RECORDER_TOPICS,
            qos_binding)


@pytest.mark.parametrize(("mutation", "failure"), (
    ("positional", "PROC_RECORDER_TOPICS_FLAG"),
    ("duplicate_flag", "PROC_RECORDER_TOPICS_FLAG"),
    ("before_include", "PROC_RECORDER_TOPICS_FLAG"),
    ("swapped", "PROC_RECORDER_TOPICS_EXACT_ORDER"),
    ("trailing", "PROC_RECORDER_TOPICS_EXACT_ORDER"),
))
def test_recorder_command_requires_topics_flag_and_exact_terminal_vector(
        mutation, failure):
    qos_binding = {
        "path": "/tmp/candidate3_qos.yaml", "mode": "0400",
        "sha256": MODULE.RECORDER_QOS_OVERRIDES_SHA256,
    }
    argv = [
        "/opt/ros/jazzy/bin/ros2", "bag", "record",
        MODULE.RECORDER_INCLUDE_UNPUBLISHED_FLAG,
        "-s", "sqlite3", "-o", "/tmp/candidate3_bag",
        MODULE.RECORDER_QOS_FLAG, qos_binding["path"],
        MODULE.RECORDER_CACHE_FLAG, str(MODULE.RECORDER_CACHE_BYTES),
        MODULE.RECORDER_TOPICS_FLAG, *MODULE.RECORDER_TOPICS,
    ]
    topics_index = argv.index(MODULE.RECORDER_TOPICS_FLAG)
    if mutation == "positional":
        argv.pop(topics_index)
    elif mutation == "duplicate_flag":
        argv.insert(topics_index, MODULE.RECORDER_TOPICS_FLAG)
    elif mutation == "before_include":
        argv.pop(topics_index)
        argv.insert(
            argv.index(MODULE.RECORDER_INCLUDE_UNPUBLISHED_FLAG),
            MODULE.RECORDER_TOPICS_FLAG)
    elif mutation == "swapped":
        argv[topics_index + 1], argv[topics_index + 2] = (
            argv[topics_index + 2], argv[topics_index + 1])
    else:
        argv.append("unexpected-trailing-argument")
    with pytest.raises(MODULE.LiveCertificateError, match=failure):
        MODULE._validate_recorder_argv(
            argv, "/tmp/candidate3_bag", MODULE.RECORDER_TOPICS,
            qos_binding)


@pytest.mark.parametrize(("mutation", "failure"), (
    ("missing_qos", "PROC_RECORDER_QOS"),
    ("wrong_qos", "PROC_RECORDER_QOS_CACHE_CONTRACT"),
    ("wrong_qos_mode", "PROC_RECORDER_QOS_CACHE_CONTRACT"),
    ("missing_cache", "PROC_RECORDER_CACHE"),
    ("wrong_cache", "PROC_RECORDER_QOS_CACHE_CONTRACT"),
    ("qos_after_topics", "PROC_RECORDER_QOS_CACHE_ORDER"),
))
def test_recorder_command_requires_bound_qos_and_exact_cache_before_topics(
        mutation, failure):
    qos_binding = {
        "path": "/tmp/candidate3_qos.yaml", "mode": "0400",
        "sha256": MODULE.RECORDER_QOS_OVERRIDES_SHA256,
    }
    argv = [
        "/opt/ros/jazzy/bin/ros2", "bag", "record",
        MODULE.RECORDER_INCLUDE_UNPUBLISHED_FLAG,
        "-s", "sqlite3", "-o", "/tmp/candidate3_bag",
        MODULE.RECORDER_QOS_FLAG, qos_binding["path"],
        MODULE.RECORDER_CACHE_FLAG, str(MODULE.RECORDER_CACHE_BYTES),
        MODULE.RECORDER_TOPICS_FLAG, *MODULE.RECORDER_TOPICS,
    ]
    if mutation == "missing_qos":
        index = argv.index(MODULE.RECORDER_QOS_FLAG)
        del argv[index:index + 2]
    elif mutation == "wrong_qos":
        argv[argv.index(MODULE.RECORDER_QOS_FLAG) + 1] = "/tmp/wrong.yaml"
    elif mutation == "wrong_qos_mode":
        qos_binding["mode"] = "0600"
    elif mutation == "missing_cache":
        index = argv.index(MODULE.RECORDER_CACHE_FLAG)
        del argv[index:index + 2]
    elif mutation == "wrong_cache":
        argv[argv.index(MODULE.RECORDER_CACHE_FLAG) + 1] = str(
            MODULE.RECORDER_CACHE_BYTES + 1)
    else:
        index = argv.index(MODULE.RECORDER_QOS_FLAG)
        qos_pair = argv[index:index + 2]
        del argv[index:index + 2]
        argv.extend(qos_pair)
    with pytest.raises(MODULE.LiveCertificateError, match=failure):
        MODULE._validate_recorder_argv(
            argv, "/tmp/candidate3_bag", MODULE.RECORDER_TOPICS,
            qos_binding)


def test_camera_and_window_identity_are_fail_closed(tmp_path):
    fixture = _fixture(tmp_path)
    _rewrite_json(
        fixture.paths.camera,
        lambda value: value.__setitem__("response_stdout", "data: false\n"))
    with pytest.raises(MODULE.LiveCertificateError, match="GUI_CAMERA_NOT_PASS"):
        _assemble(fixture)

    fixture = _fixture(tmp_path / "window")
    _rewrite_json(
        fixture.paths.window,
        lambda value: value.__setitem__("wm_class", ["other", "window"]))
    with pytest.raises(
            MODULE.LiveCertificateError,
            match="VIDEO_WINDOW_UNIQUE_IDENTITY"):
        _assemble(fixture)


@pytest.mark.parametrize(("mutation", "failure"), (
    ("missing_cube", "WORKCELL_POSE_POSTFLIGHT_0_ENTITIES_KEYS"),
    ("cube_drift", "WORKCELL_POSE_POSTFLIGHT_0_green_cube_GATE"),
    ("wrong_target", "WORKCELL_POSE_TERMINAL_CUBE_TARGET_BINDING"),
))
def test_terminal_cube_live_pose_is_fail_closed(
        tmp_path, mutation, failure):
    fixture = _fixture(tmp_path)

    def mutate(value):
        if mutation == "missing_cube":
            del value["postflight_samples"][0]["entities"]["green_cube"]
        elif mutation == "cube_drift":
            value["postflight_samples"][0]["entities"]["green_cube"][
                "position_m"][0] += 0.004
        else:
            value["terminal_cube_target"]["position_m"][0] = 0.612

    _rewrite_json(fixture.paths.workcell, mutate)
    with pytest.raises(MODULE.LiveCertificateError, match=failure):
        _assemble(fixture)


def test_support_manifest_semantically_binds_terminal_cube_target(tmp_path):
    fixture = _fixture(tmp_path)
    items = json.loads(json.dumps(fixture.authority_items))
    by_id = {item["id"]: item for item in items}

    def load(identifier):
        return json.loads(Path(by_id[identifier]["path"]).read_text(
            encoding="utf-8"))

    def reseal(identifier, value):
        item = by_id[identifier]
        path = Path(item["path"])
        path.chmod(0o600)
        raw = _canonical(value)
        path.write_bytes(raw)
        path.chmod(int(item["mode"], 8))
        item["sha256"] = hashlib.sha256(raw).hexdigest()
        item["size_bytes"] = len(raw)

    support = load("middle_support_manifest")
    support["support_contract"]["boundaries"]["CUBE_RELEASE"][
        "cube_pose"]["position"][0] = 0.612
    contract_sha = MODULE._canonical_hash(support["support_contract"])
    plan_value = load("middle_plan")
    plan_value["support_contract_sha256"] = contract_sha
    reseal("middle_plan", plan_value)
    result = load("middle_exact_result")
    result["support_contract_sha256"] = contract_sha
    result["plan_file_sha256"] = by_id["middle_plan"]["sha256"]
    reseal("middle_exact_result", result)
    certificate = load("middle_certificate")
    certificate["support_contract_sha256"] = contract_sha
    certificate["plan_sha256"] = by_id["middle_plan"]["sha256"]
    certificate["exact_result_sha256"] = by_id[
        "middle_exact_result"]["sha256"]
    reseal("middle_certificate", certificate)
    support["bindings"].update({
        "support_contract_sha256": contract_sha,
        "plan_sha256": by_id["middle_plan"]["sha256"],
        "exact_result_sha256": by_id["middle_exact_result"]["sha256"],
        "certificate_sha256": by_id["middle_certificate"]["sha256"],
    })
    reseal("middle_support_manifest", support)
    formal = {"authority_bindings": items}
    plan = json.loads(fixture.paths.plan.read_text(encoding="utf-8"))
    with pytest.raises(
            MODULE.LiveCertificateError,
            match="DELEGATION_SUPPORT_CUBE_RELEASE_TARGET"):
        MODULE._validate_dynamic_cube_delegation(formal, plan, 0.5)


def test_numeric_v4_q90_to_middle_e0_seam_is_fail_closed(tmp_path):
    fixture = _fixture(tmp_path)
    formal = _mutated_formal_summary(
        fixture, "right_manifest",
        lambda value: value["audited_state_identity"]["q90_canonical_e0"][
            "right_arm_rad"].__setitem__(0, -0.40))
    plan = json.loads(fixture.paths.plan.read_text(encoding="utf-8"))
    with pytest.raises(
            MODULE.LiveCertificateError,
            match="DELEGATION_PROJECTED_E0"):
        MODULE._validate_dynamic_cube_delegation(formal, plan, 0.5)


def test_certificate_source_self_identity_rejects_another_copy(tmp_path):
    fixture = _fixture(tmp_path)
    copy = (tmp_path / "other_certificate.py").resolve()
    digest = _sealed(copy, SOURCE.read_bytes(), 0o500)
    plan = json.loads(fixture.paths.plan.read_text(encoding="utf-8"))
    plan["evidence_tools"]["certificate_source"] = {
        "path": str(copy), "sha256": digest,
        "size_bytes": copy.stat().st_size, "mode": "0500",
    }
    with pytest.raises(
            MODULE.LiveCertificateError,
            match="PLAN_CERTIFICATE_SOURCE_SELF_PATH"):
        MODULE._validate_plan(plan)
