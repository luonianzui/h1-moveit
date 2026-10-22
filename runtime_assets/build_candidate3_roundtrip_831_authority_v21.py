#!/usr/bin/python3
"""Build the Mid360 navigation-map round-trip replay as fresh v21/r31."""

from __future__ import annotations

import ast
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import stat
import sys
from typing import Any, Mapping
import xml.etree.ElementTree as ET


HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parent
V20_BUILDER = HERE / "build_candidate3_roundtrip_831_authority_v20.py"
NAVIGATION_WORLD = (
    WORKSPACE / "src/h1_2_navigation/worlds/unitree_h12_world.sdf")
MID360_CONFIG = (
    WORKSPACE / "src/h1_2_navigation/config/mid360_localization.yaml")
MID360_LOCALIZER = (
    WORKSPACE /
    "src/h1_2_navigation/h1_2_navigation/mid360_square_localizer.py")
SQUARE_LOCALIZATION = (
    WORKSPACE / "src/h1_2_navigation/h1_2_navigation/square_localization.py")

NAVIGATION_WORLD_SHA256 = (
    "cf090dfcd59badd5cc285cd48418c1d43d6e38a1ade9699949d383fa9485648f")
MID360_CONFIG_SHA256 = (
    "4786feb493efc8a9fee306ff90d247320b42ad44823826701d247a829f7cd27f")
MID360_LOCALIZER_SHA256 = (
    "65c49409765cf9a0194ca2a9476d881b466d2dcf516ceb34deef9694d913a199")
SQUARE_LOCALIZATION_SHA256 = (
    "351f6e2eeb5c3d1cc89d28e3301a5cff5c6c22c18b94e9b07d9d0d34149b005f")
COMBINED_WORLD_SHA256 = (
    "6da0ddd8a9af867d427293b4bae1cc4a82e4c03ec9497755a9a835951b70e671")
COMBINED_LAUNCH_SHA256 = (
    "3b756c1d321e5dbd7e851031b4e1add33ee6dd4562cc2b2daee2f64598cd4492")


SPEC = importlib.util.spec_from_file_location(
    "candidate3_roundtrip_831_authority_v21_base", V20_BUILDER)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError("V21_BASE_BUILDER_IMPORT")
V20 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = V20
SPEC.loader.exec_module(V20)
BUILDER = V20.BUILDER


BASE_PREPARE_AUTHORITY = BUILDER.prepare_authority
V20_RUN_ID_EXTRA_BINDING = copy.deepcopy(BUILDER.RUN_ID_EXTRA_BINDING)
V20_AUTHORITY = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v20")
V9_AUTHORITY = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v9")


def _sha256(raw: bytes) -> str:
    return hashlib.sha256(raw).hexdigest()


def _read_bound_source(path: Path, expected_sha256: str) -> bytes:
    metadata = path.lstat()
    if not stat.S_ISREG(metadata.st_mode) or path.is_symlink():
        raise RuntimeError(f"V21_SOURCE_TYPE:{path}")
    if metadata.st_nlink != 1:
        raise RuntimeError(f"V21_SOURCE_NLINK:{path}")
    raw = path.read_bytes()
    actual_sha256 = _sha256(raw)
    if actual_sha256 != expected_sha256:
        raise RuntimeError(
            f"V21_SOURCE_SHA256:{path}:{expected_sha256}:{actual_sha256}")
    return raw


def _replace_once(text: str, old: str, new: str, label: str) -> str:
    if text.count(old) != 1:
        raise RuntimeError(f"V21_REPLACE_COUNT:{label}:{text.count(old)}")
    return text.replace(old, new)


def _build_combined_world(base_world: bytes, navigation_world: bytes) -> bytes:
    base_text = base_world.decode("utf-8")
    navigation_text = navigation_world.decode("utf-8")
    enclosure_marker = '    <model name="navigation_enclosure">'
    world_end = "  </world>"
    if navigation_text.count(enclosure_marker) != 1:
        raise RuntimeError("V21_NAVIGATION_ENCLOSURE_COUNT")
    if navigation_text.count(world_end) != 1:
        raise RuntimeError("V21_NAVIGATION_WORLD_END_COUNT")
    if "navigation_enclosure" in base_text or base_text.count(world_end) != 1:
        raise RuntimeError("V21_BASE_WORLD_STRUCTURE")
    enclosure = navigation_text[
        navigation_text.index(enclosure_marker):navigation_text.index(world_end)
    ].rstrip()
    combined = base_text.replace(world_end, enclosure + "\n" + world_end).encode()
    if _sha256(combined) != COMBINED_WORLD_SHA256:
        raise RuntimeError("V21_COMBINED_WORLD_SHA256")
    root = ET.fromstring(combined)
    world = root.find("world")
    if world is None:
        raise RuntimeError("V21_COMBINED_WORLD_MISSING")
    if world.findtext("physics/max_step_size") != "0.005":
        raise RuntimeError("V21_ACTION_PHYSICS_STEP")
    if world.findtext("physics/real_time_factor") != "1.0":
        raise RuntimeError("V21_ACTION_REAL_TIME_FACTOR")
    if [model.get("name") for model in world.findall("model")] != [
            "ground_plane", "navigation_enclosure"]:
        raise RuntimeError("V21_COMBINED_WORLD_MODELS")
    collisions = world.findall(
        "model[@name='navigation_enclosure']/link/collision")
    sizes = [collision.findtext("geometry/box/size") for collision in collisions]
    if sizes != [
            "5.2 0.1 2.5", "5.2 0.1 2.5",
            "0.1 5.0 2.5", "0.1 5.0 2.5"]:
        raise RuntimeError("V21_NAVIGATION_WALL_GEOMETRY")
    return combined


def _build_combined_launch(base_launch: bytes) -> bytes:
    text = base_launch.decode("utf-8")
    telemetry_constant = (
        "TELEMETRY_BRIDGE_CONFIG_SHA256 = (\n"
        "    '615231d3fc7d810f38165cbc679e4044263d4000cace8b56cc0ed35705ec9b0d')\n"
    )
    text = _replace_once(
        text,
        telemetry_constant,
        telemetry_constant +
        "MID360_LOCALIZATION_CONFIG_SHA256 = (\n"
        f"    '{MID360_CONFIG_SHA256}')\n",
        "mid360_config_constant",
    )
    text = _replace_once(
        text,
        "    world_file = PathJoinSubstitution([\n"
        "        FindPackageShare('h1_candidate3_fast_831'), 'worlds', 'h1_world.sdf'])",
        "    world_file = os.path.join(\n"
        "        os.path.dirname(os.path.realpath(__file__)), 'h1_world.sdf')",
        "authority_local_world",
    )
    config_anchor = (
        "    realsense_description_abs_path = "
        "get_package_share_directory('realsense2_description')\n"
    )
    text = _replace_once(
        text,
        config_anchor,
        config_anchor +
        "    authority_root = os.path.dirname(os.path.realpath(__file__))\n"
        "    mid360_localization_config = os.path.join(\n"
        "        authority_root, 'mid360_localization.yaml')\n"
        "    if (_sha256_file(mid360_localization_config) !=\n"
        "            MID360_LOCALIZATION_CONFIG_SHA256):\n"
        "        raise RuntimeError(\n"
        "            'sealed Mid360 localization config changed')\n",
        "mid360_config_path",
    )
    support_anchor = (
        "    # The mature support preloads require their Gazebo transport topics to\n"
    )
    mid360_nodes = (
        "    # Keep the completed Mid360 square-map perception active in the sealed\n"
        "    # manipulation scene. These nodes are observational only and publish no\n"
        "    # trajectory, support, door, gripper, or Gazebo command.\n"
        "    mid360_point_cloud_bridge = Node(\n"
        "        package='ros_gz_bridge',\n"
        "        executable='parameter_bridge',\n"
        "        arguments=[\n"
        "            '/livox/mid360/points@'\n"
        "            'sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',\n"
        "        ],\n"
        "        output='screen',\n"
        "    )\n"
        "    mid360_localizer = Node(\n"
        "        package='h1_2_navigation',\n"
        "        executable='mid360_square_localizer',\n"
        "        output='screen',\n"
        "        parameters=[mid360_localization_config, {\n"
        "            'use_sim_time': use_sim_time,\n"
        "            'publish_map_to_odom': False,\n"
        "            'initial_x': 0.0,\n"
        "            'initial_y': 0.0,\n"
        "            'initial_yaw': 0.0,\n"
        "            'static_obstacle_enabled': True,\n"
        "            'static_obstacle_minimum_x': 0.20,\n"
        "            'static_obstacle_maximum_x': 1.00,\n"
        "            'static_obstacle_minimum_y': -0.60,\n"
        "            'static_obstacle_maximum_y': 0.60,\n"
        "        }],\n"
        "    )\n\n"
    )
    text = _replace_once(
        text, support_anchor, mid360_nodes + support_anchor, "mid360_nodes")
    text = _replace_once(
        text,
        "        gz_ros_bridge,\n"
        "        support_state_bridge,",
        "        gz_ros_bridge,\n"
        "        mid360_point_cloud_bridge,\n"
        "        mid360_localizer,\n"
        "        support_state_bridge,",
        "mid360_launch_actions",
    )
    ast.parse(text, filename="gazebo.launch.py")
    combined = text.encode("utf-8")
    if _sha256(combined) != COMBINED_LAUNCH_SHA256:
        raise RuntimeError("V21_COMBINED_LAUNCH_SHA256")
    return combined


def prepare_authority() -> Any:
    navigation_world = _read_bound_source(
        NAVIGATION_WORLD, NAVIGATION_WORLD_SHA256)
    mid360_config = _read_bound_source(MID360_CONFIG, MID360_CONFIG_SHA256)
    _read_bound_source(MID360_LOCALIZER, MID360_LOCALIZER_SHA256)
    _read_bound_source(SQUARE_LOCALIZATION, SQUARE_LOCALIZATION_SHA256)
    prepared = BASE_PREPARE_AUTHORITY()
    output_files = dict(prepared.output_files)
    output_files["h1_world.sdf"] = BUILDER.OutputBlob(
        _build_combined_world(
            prepared.output_files["h1_world.sdf"].raw, navigation_world),
        0o400,
        "generated:v20-action-world+bound-navigation-enclosure",
    )
    output_files["gazebo.launch.py"] = BUILDER.OutputBlob(
        _build_combined_launch(
            prepared.output_files["gazebo.launch.py"].raw),
        0o400,
        "generated:v20-action-launch+mid360-map-observers",
    )
    output_files["mid360_localization.yaml"] = BUILDER.OutputBlob(
        mid360_config,
        0o400,
        BUILDER._relative(MID360_CONFIG),
    )

    manifest = dict(prepared.manifest)
    contract = dict(manifest["execution_contract"])
    contract["live_demonstration_instance"] = "r31"
    contract["combined_mid360_navigation_map_scene"] = True
    contract["navigation_enclosure_wall_count"] = 4
    contract["navigation_enclosure_half_extent_m"] = 2.5
    contract["navigation_enclosure_height_m"] = 2.5
    contract["mid360_ros_point_cloud_enabled"] = True
    contract["mid360_square_localization_enabled"] = True
    contract["action_physics_max_step_s"] = 0.005
    contract["action_physics_unchanged_from_v9"] = True
    contract["environment_authority_differs_from_v9"] = True
    contract["action_payloads_unchanged_except_scene_launch"] = True
    contract["identity_independent_action_payload_count"] = 14
    manifest["execution_contract"] = contract

    derived = dict(manifest["derived_from"])
    derived["semantic_changes"] = [
        "allocate a fresh non-overwriting r31 replay identity",
        "retain all 14 non-scene action, timing, and safety payloads exactly",
        "bind the exact completed 5 m Mid360 navigation enclosure geometry",
        "retain the proven 5 ms action physics and manipulation coordinates",
        "publish Mid360 ROS point cloud, scan, map, pose, and quality status",
        "record the complete natural-speed replay with rosbag and GUI video",
    ]
    manifest["derived_from"] = derived
    manifest["combined_scene_binding"] = {
        "source_navigation_world": {
            "path": BUILDER._relative(NAVIGATION_WORLD),
            "sha256": NAVIGATION_WORLD_SHA256,
        },
        "source_mid360_config": {
            "path": BUILDER._relative(MID360_CONFIG),
            "sha256": MID360_CONFIG_SHA256,
        },
        "source_mid360_localizer_sha256": MID360_LOCALIZER_SHA256,
        "source_square_localization_sha256": SQUARE_LOCALIZATION_SHA256,
        "combined_world_sha256": COMBINED_WORLD_SHA256,
        "combined_launch_sha256": COMBINED_LAUNCH_SHA256,
        "wall_geometry_identical_to_navigation_world": True,
        "action_physics_identical_to_v20": True,
        "observational_mid360_pipeline_only": True,
    }
    files = dict(manifest["files"])
    provenance = dict(manifest["provenance"])
    for name in (
            "h1_world.sdf", "gazebo.launch.py", "mid360_localization.yaml"):
        files[name] = BUILDER._output_binding(name, output_files[name])
        provenance[name] = output_files[name].provenance
    manifest["files"] = files
    manifest["provenance"] = provenance
    output_files["manifest.json"] = BUILDER.OutputBlob(
        BUILDER.pretty(manifest), 0o400, "generated")
    return BUILDER.PreparedAuthority(
        run_id=prepared.run_id,
        socket_path=prepared.socket_path,
        output_files=output_files,
        commands=prepared.commands,
        return_authorization=prepared.return_authorization,
        manifest=manifest,
    )


def self_test() -> Mapping[str, Any]:
    prepared = prepare_authority()
    all_payloads = (
        V20.V19.V18.V17.V16.V15.V14.V13_IDENTITY_INDEPENDENT_SHA256)
    action_payloads = {
        name: digest for name, digest in all_payloads.items()
        if name not in {"gazebo.launch.py", "h1_world.sdf"}
    }
    for name, expected in action_payloads.items():
        actual = _sha256(prepared.output_files[name].raw)
        if actual != expected:
            raise RuntimeError(f"V21_SELF_TEST_ACTION_PAYLOAD_DRIFT:{name}")
    if len(action_payloads) != 14:
        raise RuntimeError("V21_SELF_TEST_ACTION_PAYLOAD_COUNT")
    if (_sha256(prepared.output_files["gazebo.launch.py"].raw) !=
            COMBINED_LAUNCH_SHA256):
        raise RuntimeError("V21_SELF_TEST_LAUNCH")
    if (_sha256(prepared.output_files["h1_world.sdf"].raw) !=
            COMBINED_WORLD_SHA256):
        raise RuntimeError("V21_SELF_TEST_WORLD")
    if (_sha256(prepared.output_files["mid360_localization.yaml"].raw) !=
            MID360_CONFIG_SHA256):
        raise RuntimeError("V21_SELF_TEST_MID360_CONFIG")
    v20_commands = json.loads(
        (V20_AUTHORITY / "commands_831.json").read_text(encoding="utf-8"))
    if prepared.commands["timing_contract"] != v20_commands["timing_contract"]:
        raise RuntimeError("V21_SELF_TEST_TIMING_DRIFT")
    launch_text = prepared.output_files["gazebo.launch.py"].raw.decode()
    if "mid360_point_cloud_bridge" not in launch_text:
        raise RuntimeError("V21_SELF_TEST_POINT_CLOUD_BRIDGE")
    if "mid360_localizer" not in launch_text:
        raise RuntimeError("V21_SELF_TEST_LOCALIZER")
    if "os.path.realpath(__file__)" not in launch_text:
        raise RuntimeError("V21_SELF_TEST_AUTHORITY_LOCAL_WORLD")
    return {
        "schema": "h1_candidate3_roundtrip_builder_self_test/831-v21",
        "run_id": prepared.run_id,
        "authority": str(BUILDER.NEW),
        "run_root": str(BUILDER.RUN_ROOT),
        "semantic_action_authority_identical_to_v9": True,
        "environment_authority_differs_from_v9": True,
        "combined_mid360_navigation_map_scene": True,
        "identity_independent_action_payloads_checked": len(action_payloads),
        "action_payloads_match_v9": all(
            _sha256(prepared.output_files[name].raw) ==
            _sha256((V9_AUTHORITY / name).read_bytes())
            for name in action_payloads),
        "timing_contract_identical_to_v20": True,
        "controller_commands_issued": 0,
        "writes_performed": False,
        "status": "PASS",
        "valid": True,
    }


BUILDER.NEW = (
    BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_authority_v21")
BUILDER.RUN_ROOT = BUILDER.RUNTIME_ASSETS / "candidate3_roundtrip_831_r31"
BUILDER.AUTHORITY_SCHEMA = "h1_candidate3_roundtrip_authority/831-v21"
BUILDER.COMMANDS_SCHEMA = "h1_candidate3_roundtrip_commands/831-v21"
BUILDER.RUN_ID_EXTRA_BINDING = V20_RUN_ID_EXTRA_BINDING
BUILDER.RUN_ID_EXTRA_BINDING.pop("post_backup_914_full_replay_v20", None)
BUILDER.RUN_ID_EXTRA_BINDING["combined_mid360_map_replay_v21"] = {
    "instance": "r31",
    "semantic_authority_manifest_sha256":
        V20.V19.V18.V17.V16.V15.V14.V13.V12.V11.V9_MANIFEST_SHA256,
    "base_replay_manifest_sha256":
        "41848637a6f1b117dec975722e3dc789faff9f6b7394df6a87157c663a4f4c32",
    "navigation_world_sha256": NAVIGATION_WORLD_SHA256,
    "mid360_config_sha256": MID360_CONFIG_SHA256,
    "mid360_localizer_sha256": MID360_LOCALIZER_SHA256,
    "square_localization_sha256": SQUARE_LOCALIZATION_SHA256,
    "combined_world_sha256": COMBINED_WORLD_SHA256,
    "combined_launch_sha256": COMBINED_LAUNCH_SHA256,
    "action_payload_count_unchanged": 14,
    "action_trajectory_and_timing_unchanged": True,
    "action_physics_unchanged": True,
    "required_rosbag_present": True,
    "required_gui_video_present": True,
}
BUILDER.prepare_authority = prepare_authority
BUILDER.self_test = self_test


if __name__ == "__main__":
    raise SystemExit(BUILDER.main())
