"""
Pure-data contract for the right-open/left-cube/right-close middle flow.

This module deliberately does not import ROS and cannot publish trajectories.
It binds the only three admitted 826.3 assets, the exact replay splits used by
``h1_closed_loop_90_8254.sh right-middle-run``, and the full-body hand-off
state shared with the right-hand handle prefix/suffix.

The dynamic exact-geometry audit is a separate fail-closed layer.  A plan made
here is only an audit input; it is never an execution certificate by itself.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Mapping, Sequence


PLAN_SCHEMA = 'h1_candidate3_middle_plan/v1'
CERTIFICATE_SCHEMA = 'h1_candidate3_middle_certificate/v1'
AUDIT_EVIDENCE_SCHEMA = 'h1_candidate3_middle_exact_audit/v1'
WORKER_VERSION = 'h1_candidate3_middle_worker/828-a1'
AUDITOR_VERSION = 'h1_candidate3_middle_exact_auditor/828-a1'
TOPOLOGY = 'right_door_open_left_cube_right_door_close_v1'

HANDLE_EXACT_ID = 'incubator_handle_exact'
HANDLE_STL_SHA256 = (
    'ad6670185fa5756ba5dabdee7e542db2df3346973fea4eb887288f54a6b6f59e')
HANDLE_ANGLE_RAD = math.pi / 2.0
DOOR_OPEN_ANGLE_RAD = math.pi / 3.0
REQUIRED_CLEARANCE_M = 0.010
MAXIMUM_ACTION_DURATION_S = 90.0
MASK_CONFLICT_PAIR = (
    'incubator_handle_visual_mask',
    'left_dex1_finger_2_visual_mask',
)
MASK_OBSERVED_DISTANCE_M = 0.002665455
MASK_RECHECK_DENSE_SCOPE = 'cube_asset_global_release_recheck_grid_v1'
MASK_RECHECK_ANGULAR_STEP_RAD = math.radians(3.0)
MASK_RECHECK_PRISMATIC_STEP_M = 0.0001
PERCEPTION_ONLY_MASK_IDS = (
    'incubator_handle_visual_mask',
    'incubator_handle_visual_mask_attached',
    'left_wrist_yaw_visual_mask',
    'right_wrist_yaw_visual_mask',
    'left_ankle_pitch_visual_mask',
    'right_ankle_pitch_visual_mask',
    'left_ankle_roll_visual_mask',
    'right_ankle_roll_visual_mask',
    'left_knee_visual_mask',
    'right_knee_visual_mask',
    'left_dex1_finger_2_slider_visual_mask',
    'left_dex1_finger_2_visual_mask',
    'left_dex1_finger_1_visual_mask',
)
PHYSICAL_LEFT_FINGER_2_LINKS = (
    'left_dex1_finger_2_slider_link',
    'left_dex1_finger_2_link',
    'left_dex1_finger_2_pad_link',
)
RIGHT_HANDLE_ALLOWED_CONTACT_PAIRS = (
    (HANDLE_EXACT_ID, 'right_dex1_finger_1_pad_link'),
    (HANDLE_EXACT_ID, 'right_dex1_finger_2_pad_link'),
)

# The standalone C++ producer accepts either an already-physical frozen CDR or
# a raw CDR containing the complete known conflict-mask pair.  In the latter
# case it removes only the exact allowlist.  A partial pair or any unknown mask
# fails closed.  Both branches retain the one-mesh/zero-primitive q90 handle
# and calculate the same cube-release exact-STL neighborhood.


class MiddleContractError(ValueError):
    """Raised when an input cannot belong to the middle-only topology."""


def canonical_bytes(value) -> bytes:
    """Return the sole canonical JSON encoding used by middle-v1."""
    return json.dumps(
        value, sort_keys=True, separators=(',', ':'), ensure_ascii=True,
        allow_nan=False).encode('utf-8')


def canonical_hash(value) -> str:
    """Hash a value using the middle-v1 canonical JSON encoding."""
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def file_sha256(path: Path) -> str:
    """Return the SHA-256 of a file without loading it all into memory."""
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        while True:
            block = stream.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


@dataclass(frozen=True)
class AssetSpec:
    """Immutable identity and basic structure of one admitted asset."""

    logical_name: str
    file_name: str
    stage: str
    message_count: int
    payload_sha256: str
    file_sha256: str


ASSET_SPECS = (
    AssetSpec(
        'door_open', 'door.trajectory.json', 'door', 13,
        '867548548197ae577580f394051e8118cc0fccefdea1494c1bbb41ccaa4a4a19',
        'c481657ac205017a5be16646532236c97dedf909eb0571297e70f9d8690131c9'),
    AssetSpec(
        'cube', 'cube.parallel90.neutral.trajectory.json', 'cube', 6,
        'acaec97384f16a736f6b246a3fa1a0897b7aad93e72a6204701610ebf4f7e705',
        '09919a92a65b5ca90b3a11420bc9209ceead1e90112f94da2c4c18711af8e134'),
    AssetSpec(
        'door_close', 'door_close.trajectory.json', 'door', 13,
        '5b212d78bb204fdde047c875d1e2e9585efb18d7d053c847be0a0ef59a4a9038',
        '3cbb2decfe3128e6a7ca95238085f4285b06a79486c40733db7d426d0a3e5afb'),
)
ASSET_SPEC_BY_NAME = {value.logical_name: value for value in ASSET_SPECS}
SOURCE_ASSET_HASHES = {
    value.logical_name: value.payload_sha256 for value in ASSET_SPECS
}
SOURCE_FILE_HASHES = {
    value.logical_name: value.file_sha256 for value in ASSET_SPECS
}
SOURCE_CACHE_HASH = (
    'dfc45b9ab0ad12e6a6e038ed0a2a3300838ec143ad8dd4c591b423273a352a02')

# These stages belong to the legacy left-handle topology.  Merely replacing
# them with empty messages would preserve the wrong semantics, so middle-v1
# forbids their names and their source payloads entirely.
FORBIDDEN_LEGACY_STAGES = (
    'handle_unlock', 'left_clear', 'left_regrasp', 'handle_relock',
)
FORBIDDEN_LEGACY_ASSET_HASHES = (
    'c03df7c51daac6d2059ab5316d17dd9d486191d1bc2aadd0395cfe73044adc6b',
    '066b2437fa320d2db895a927e24f0d5044ff2a9786638e335f8117695e105b88',
    '5acd13b1a497b2496ca50d06982316d18b4d482a3288126e1a66fe36f8fd6f2d',
    'b6b15a9ec270cb3158e0c815997cfa85ebefb04ba3cd3d878c5ed1ff8b9b5ce3',
)


LEFT_ARM_JOINTS = (
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint',
    'left_shoulder_yaw_joint', 'left_elbow_joint',
    'left_wrist_roll_joint', 'left_wrist_pitch_joint',
    'left_wrist_yaw_joint',
)
RIGHT_ARM_JOINTS = tuple(value.replace('left_', 'right_', 1)
                         for value in LEFT_ARM_JOINTS)
LEFT_GRIPPER_JOINTS = (
    'left_dex1_finger_1_joint', 'left_dex1_finger_2_joint',
)
RIGHT_GRIPPER_JOINTS = (
    'right_dex1_finger_1_joint', 'right_dex1_finger_2_joint',
)

SEAM_JOINT_NAMES = (
    *LEFT_ARM_JOINTS, *RIGHT_ARM_JOINTS,
    *LEFT_GRIPPER_JOINTS, *RIGHT_GRIPPER_JOINTS, 'torso_joint',
)
SEAM_NOMINAL = (
    0.0, 0.30, 0.0, 0.60, 0.0, 0.0, 0.0,
    0.0, -0.25, 0.0, 0.60, 0.0, 0.0, 0.0,
    -0.019, -0.019, -0.019, -0.019, 0.0,
)
SEAM_TOLERANCES = (
    *(0.004 for _ in range(14)),
    *(0.001 for _ in range(4)),
    0.003,
)
SEAM_CONTRACT = {
    'schema': 'h1_candidate3_middle_seam/v1',
    'joint_names': list(SEAM_JOINT_NAMES),
    'nominal_positions': list(SEAM_NOMINAL),
    'absolute_tolerances': list(SEAM_TOLERANCES),
    'torso_controller_authorized': False,
}
SEAM_STATE_HASH = canonical_hash(SEAM_CONTRACT)


@dataclass(frozen=True)
class PhaseSpec:
    """One executable range inside an admitted middle asset."""

    name: str
    asset: str
    start: int
    end: int
    hand: str
    door_start_rad: float
    door_end_rad: float
    door_support: str
    cube_support: str


PHASE_SPECS = (
    PhaseSpec('door_pre_support', 'door_open', 0, 6, 'right',
              0.0, 0.0, 'detached', 'detached'),
    PhaseSpec('door_open_supported', 'door_open', 6, 13, 'right',
              0.0, DOOR_OPEN_ANGLE_RAD, 'attached', 'detached'),
    PhaseSpec('cube_contact', 'cube', 0, 2, 'left',
              DOOR_OPEN_ANGLE_RAD, DOOR_OPEN_ANGLE_RAD,
              'attached', 'detached'),
    PhaseSpec('cube_carry_place', 'cube', 2, 4, 'left',
              DOOR_OPEN_ANGLE_RAD, DOOR_OPEN_ANGLE_RAD,
              'attached', 'attached'),
    PhaseSpec('cube_release', 'cube', 4, 5, 'left',
              DOOR_OPEN_ANGLE_RAD, DOOR_OPEN_ANGLE_RAD,
              'attached', 'detached'),
    PhaseSpec('cube_retreat', 'cube', 5, 6, 'left',
              DOOR_OPEN_ANGLE_RAD, DOOR_OPEN_ANGLE_RAD,
              'attached', 'detached'),
    PhaseSpec('door_close_supported', 'door_close', 0, 7, 'right',
              DOOR_OPEN_ANGLE_RAD, 0.0, 'attached', 'detached'),
    PhaseSpec('right_release', 'door_close', 7, 8, 'right',
              0.0, 0.0, 'detached', 'detached'),
    PhaseSpec('right_retreat', 'door_close', 8, 13, 'right',
              0.0, 0.0, 'detached', 'detached'),
)
PHASE_NAMES = tuple(value.name for value in PHASE_SPECS)


def default_asset_paths(workspace: Path) -> dict[str, Path]:
    """Return the authoritative three asset paths below a workspace."""
    workspace = Path(workspace)
    return {
        'door_open': workspace / 'runtime_assets/full_robot_flow_8251/'
        'door.trajectory.json',
        'cube': workspace / 'runtime_assets/parallel_cube_8262/'
        'cube.parallel90.neutral.trajectory.json',
        'door_close': workspace / 'runtime_assets/close_and_relock_8253/'
        'door_close.trajectory.json',
    }


def _require(condition: bool, status: str) -> None:
    if not condition:
        raise MiddleContractError(status)


def _finite_positions(point, width: int, status: str) -> None:
    positions = point.get('positions')
    _require(isinstance(positions, list) and len(positions) == width, status)
    _require(all(isinstance(value, (int, float)) and math.isfinite(value)
                 for value in positions), status)


def _validate_message(message, index: int, hand: str) -> None:
    _require(isinstance(message, dict), 'ASSET_MESSAGE_TYPE')
    _require(message.get('sequence') == index, 'ASSET_MESSAGE_SEQUENCE')
    topic = message.get('topic')
    _require(isinstance(topic, str), 'ASSET_MESSAGE_TOPIC')
    _require(topic.startswith(f'/{hand}_'), 'ASSET_WRONG_HAND_TOPIC')
    _require('torso' not in topic, 'ASSET_TORSO_TOPIC')
    names = message.get('joint_names')
    _require(isinstance(names, list) and len(names) in (2, 7),
             'ASSET_JOINT_NAMES')
    expected = ({
        ('left', 7): LEFT_ARM_JOINTS,
        ('left', 2): LEFT_GRIPPER_JOINTS,
        ('right', 7): RIGHT_ARM_JOINTS,
        ('right', 2): RIGHT_GRIPPER_JOINTS,
    })[(hand, len(names))]
    _require(tuple(names) == expected, 'ASSET_WRONG_HAND_JOINTS')
    points = message.get('points')
    _require(isinstance(points, list) and points, 'ASSET_EMPTY_MESSAGE')
    for point in points:
        _finite_positions(point, len(names), 'ASSET_POINT_POSITIONS')


def load_middle_assets(paths: Mapping[str, Path]) -> dict[str, dict]:
    """Load and validate exactly the three admitted immutable assets."""
    _require(set(paths) == set(ASSET_SPEC_BY_NAME), 'ASSET_SET')
    output = {}
    for logical_name, spec in ASSET_SPEC_BY_NAME.items():
        path = Path(paths[logical_name])
        _require(path.is_file() and not path.is_symlink(),
                 f'ASSET_PATH:{logical_name}')
        _require(file_sha256(path) == spec.file_sha256,
                 f'ASSET_FILE_HASH:{logical_name}')
        try:
            document = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            raise MiddleContractError(
                f'ASSET_JSON:{logical_name}') from error
        _require(set(document) == {'payload', 'sha256'},
                 f'ASSET_WRAPPER:{logical_name}')
        payload = document['payload']
        _require(isinstance(payload, dict), f'ASSET_PAYLOAD:{logical_name}')
        payload_hash = canonical_hash(payload)
        _require(document['sha256'] == payload_hash == spec.payload_sha256,
                 f'ASSET_PAYLOAD_HASH:{logical_name}')
        _require(payload.get('schema') == 'h1_joint_trajectory_asset/v1',
                 f'ASSET_SCHEMA:{logical_name}')
        _require(payload.get('stage') == spec.stage,
                 f'ASSET_STAGE:{logical_name}')
        messages = payload.get('messages')
        _require(isinstance(messages, list) and
                 len(messages) == spec.message_count,
                 f'ASSET_MESSAGE_COUNT:{logical_name}')
        hand = 'left' if logical_name == 'cube' else 'right'
        for index, message in enumerate(messages):
            _validate_message(message, index, hand)
        output[logical_name] = {
            'logical_name': logical_name,
            'file_name': spec.file_name,
            'file_sha256': spec.file_sha256,
            'payload_sha256': payload_hash,
            'payload': payload,
        }
    cube_contract = output['cube']['payload'].get('cube_cache_contract', {})
    _require(cube_contract.get('cache_sha256') == SOURCE_CACHE_HASH,
             'CUBE_CACHE_HASH')
    return output


def _group_positions(payload, width: int) -> list[list[float]]:
    return [point['positions']
            for message in payload['messages']
            if len(message['joint_names']) == width
            for point in message['points']]


def _close(
        left: Sequence[float], right: Sequence[float],
        tolerance: float) -> bool:
    return len(left) == len(right) and all(
        abs(float(a) - float(b)) <= tolerance
        for a, b in zip(left, right))


def _validate_asset_topology(assets: Mapping[str, dict]) -> None:
    door = assets['door_open']['payload']
    close = assets['door_close']['payload']
    for width in (2, 7):
        forward = _group_positions(door, width)
        reverse = _group_positions(close, width)
        _require(len(forward) == len(reverse), 'DOOR_REVERSE_POINT_COUNT')
        _require(all(_close(a, b, 1e-12)
                     for a, b in zip(forward, reversed(reverse))),
                 'DOOR_REVERSE_TRAJECTORY')

    seam = dict(zip(SEAM_JOINT_NAMES, SEAM_NOMINAL))
    tolerance = dict(zip(SEAM_JOINT_NAMES, SEAM_TOLERANCES))
    endpoint_checks = (
        (door, 'first', RIGHT_ARM_JOINTS),
        (door, 'first', RIGHT_GRIPPER_JOINTS),
        (assets['cube']['payload'], 'first', LEFT_ARM_JOINTS),
        (assets['cube']['payload'], 'first', LEFT_GRIPPER_JOINTS),
        (assets['cube']['payload'], 'last', LEFT_ARM_JOINTS),
        (assets['cube']['payload'], 'last', LEFT_GRIPPER_JOINTS),
        (close, 'last', RIGHT_ARM_JOINTS),
        (close, 'last', RIGHT_GRIPPER_JOINTS),
    )
    for payload, endpoint, names in endpoint_checks:
        matches = [message for message in payload['messages']
                   if tuple(message['joint_names']) == tuple(names)]
        _require(bool(matches), 'SEAM_GROUP_MISSING')
        message = matches[0] if endpoint == 'first' else matches[-1]
        point = (message['points'][0] if endpoint == 'first'
                 else message['points'][-1])
        for name, value in zip(names, point['positions']):
            _require(abs(float(value) - seam[name]) <= tolerance[name],
                     f'SEAM_ENDPOINT:{name}')


def validate_seam_positions(positions: Mapping[str, float]) -> None:
    """Fail closed unless a measured full seam is inside every tolerance."""
    _require(set(positions) == set(SEAM_JOINT_NAMES), 'SEAM_JOINT_SET')
    for name, expected, tolerance in zip(
            SEAM_JOINT_NAMES, SEAM_NOMINAL, SEAM_TOLERANCES):
        value = positions[name]
        _require(isinstance(value, (int, float)) and math.isfinite(value),
                 f'SEAM_NONFINITE:{name}')
        _require(abs(float(value) - expected) <= tolerance,
                 f'SEAM_POSITION:{name}')


def _phase_record(spec: PhaseSpec, assets: Mapping[str, dict]) -> dict:
    messages = assets[spec.asset]['payload']['messages'][spec.start:spec.end]
    _require(len(messages) == spec.end - spec.start,
             f'PHASE_RANGE:{spec.name}')
    expected_prefix = f'/{spec.hand}_'
    _require(all(message['topic'].startswith(expected_prefix)
                 for message in messages), f'PHASE_HAND:{spec.name}')
    record = {
        'name': spec.name,
        'source_asset': spec.asset,
        'message_range': [spec.start, spec.end],
        'authorized_hand': spec.hand,
        'authorized_topics': [message['topic'] for message in messages],
        'source_message_hashes': [
            canonical_hash(message) for message in messages],
        'door_angle_start_rad': spec.door_start_rad,
        'door_angle_end_rad': spec.door_end_rad,
        'handle_angle_rad': HANDLE_ANGLE_RAD,
        'handle_exact_id': HANDLE_EXACT_ID,
        'handle_exact_retained': True,
        'door_support': spec.door_support,
        'cube_support': spec.cube_support,
    }
    record['phase_hash'] = canonical_hash(record)
    return record


def build_middle_plan(
        assets: Mapping[str, dict], *, scene_epoch_id: int,
        raw_scene_hash: str, physical_scene_hash: str) -> dict:
    """Build a self-describing audit input; no validity claim is made here."""
    _require(set(assets) == set(ASSET_SPEC_BY_NAME), 'ASSET_SET')
    _require(isinstance(scene_epoch_id, int) and scene_epoch_id >= 0,
             'SCENE_EPOCH')
    for name, value in (('RAW_SCENE_HASH', raw_scene_hash),
                        ('PHYSICAL_SCENE_HASH', physical_scene_hash)):
        _require(isinstance(value, str) and len(value) == 64 and
                 all(character in '0123456789abcdef' for character in value),
                 name)
    _validate_asset_topology(assets)
    phases = [_phase_record(spec, assets) for spec in PHASE_SPECS]
    _require(not (set(PHASE_NAMES) & set(FORBIDDEN_LEGACY_STAGES)),
             'FORBIDDEN_STAGE')
    _require(all(value['authorized_hand'] == 'right'
                 for value in phases if value['source_asset'] != 'cube'),
             'LEFT_HANDLE_STAGE')
    _require(all(value['authorized_hand'] == 'left'
                 for value in phases if value['source_asset'] == 'cube'),
             'CUBE_HAND_STAGE')

    dynamic_schedule = [{
        'phase': value['name'],
        'door_angle_start_rad': value['door_angle_start_rad'],
        'door_angle_end_rad': value['door_angle_end_rad'],
        'handle_angle_rad': value['handle_angle_rad'],
        'handle_exact_id': value['handle_exact_id'],
        'handle_exact_retained': value['handle_exact_retained'],
        'door_support': value['door_support'],
        'cube_support': value['cube_support'],
    } for value in phases]
    plan = {
        'schema': PLAN_SCHEMA,
        'worker_version': WORKER_VERSION,
        'topology': TOPOLOGY,
        'scene_epoch_id': scene_epoch_id,
        'raw_scene_hash': raw_scene_hash,
        'physical_scene_hash': physical_scene_hash,
        'source_asset_hashes': copy.deepcopy(SOURCE_ASSET_HASHES),
        'source_file_hashes': copy.deepcopy(SOURCE_FILE_HASHES),
        'source_cache_hash': SOURCE_CACHE_HASH,
        'stage_names': list(PHASE_NAMES),
        'phases': phases,
        'phase_hashes': [value['phase_hash'] for value in phases],
        'excluded_legacy_stage_names': list(FORBIDDEN_LEGACY_STAGES),
        'left_handle_stage_count': 0,
        'seam_contract': copy.deepcopy(SEAM_CONTRACT),
        'seam_state_hash': SEAM_STATE_HASH,
        'torso_controller_authorized': False,
        'dynamic_object_schedule': dynamic_schedule,
        'dynamic_object_schedule_hash': canonical_hash(dynamic_schedule),
        'required_environment_clearance_m': REQUIRED_CLEARANCE_M,
        'contains_partial': False,
        'controller_commands_issued': 0,
        # A later exact-FCL worker must bind its evidence hash before an
        # execution certificate can be emitted.  This requirement remains in
        # the plan even after the separate evidence passes.
        'exact_geometry_audit_requirement': 'EXTERNAL_EVIDENCE_REQUIRED',
    }
    return plan


def validate_middle_plan(plan: Mapping) -> None:
    """Validate the immutable semantics of a built middle-v1 audit plan."""
    _require(isinstance(plan, Mapping), 'PLAN_TYPE')
    _require(plan.get('schema') == PLAN_SCHEMA, 'PLAN_SCHEMA')
    _require(plan.get('worker_version') == WORKER_VERSION, 'PLAN_VERSION')
    _require(plan.get('topology') == TOPOLOGY, 'PLAN_TOPOLOGY')
    _require(isinstance(plan.get('scene_epoch_id'), int) and
             plan['scene_epoch_id'] >= 0, 'PLAN_SCENE_EPOCH')
    for status, key in (
            ('PLAN_RAW_SCENE_HASH', 'raw_scene_hash'),
            ('PLAN_PHYSICAL_SCENE_HASH', 'physical_scene_hash')):
        value = plan.get(key)
        _require(isinstance(value, str) and len(value) == 64 and
                 all(character in '0123456789abcdef'
                     for character in value), status)
    _require(plan.get('source_asset_hashes') == SOURCE_ASSET_HASHES,
             'PLAN_SOURCE_ASSETS')
    _require(plan.get('source_file_hashes') == SOURCE_FILE_HASHES,
             'PLAN_SOURCE_FILES')
    _require(plan.get('source_cache_hash') == SOURCE_CACHE_HASH,
             'PLAN_SOURCE_CACHE')
    _require(plan.get('stage_names') == list(PHASE_NAMES),
             'PLAN_STAGE_NAMES')
    _require(plan.get('excluded_legacy_stage_names') ==
             list(FORBIDDEN_LEGACY_STAGES), 'PLAN_EXCLUDED_STAGES')
    _require(plan.get('left_handle_stage_count') == 0,
             'PLAN_LEFT_HANDLE_STAGE')
    _require(plan.get('seam_contract') == SEAM_CONTRACT, 'PLAN_SEAM')
    _require(plan.get('seam_state_hash') == SEAM_STATE_HASH,
             'PLAN_SEAM_HASH')
    _require(plan.get('torso_controller_authorized') is False,
             'PLAN_TORSO_CONTROLLER')
    _require(plan.get('contains_partial') is False, 'PLAN_PARTIAL')
    _require(plan.get('controller_commands_issued') == 0,
             'PLAN_CONTROLLER_COMMANDS')
    _require(plan.get('exact_geometry_audit_requirement') ==
             'EXTERNAL_EVIDENCE_REQUIRED', 'PLAN_EXACT_AUDIT_REQUIREMENT')
    phases = plan.get('phases')
    _require(isinstance(phases, list) and len(phases) == len(PHASE_SPECS),
             'PLAN_PHASE_COUNT')
    for phase, spec in zip(phases, PHASE_SPECS):
        _require(phase.get('name') == spec.name, 'PLAN_PHASE_NAME')
        _require(phase.get('source_asset') == spec.asset,
                 'PLAN_PHASE_ASSET')
        _require(phase.get('message_range') == [spec.start, spec.end],
                 'PLAN_PHASE_RANGE')
        _require(phase.get('authorized_hand') == spec.hand,
                 'PLAN_PHASE_HAND')
        topics = phase.get('authorized_topics')
        _require(isinstance(topics, list) and
                 len(topics) == spec.end - spec.start and
                 all(isinstance(topic, str) and
                     topic.startswith(f'/{spec.hand}_') and
                     'torso' not in topic for topic in topics),
                 'PLAN_PHASE_TOPICS')
        message_hashes = phase.get('source_message_hashes')
        _require(isinstance(message_hashes, list) and
                 len(message_hashes) == spec.end - spec.start and
                 all(isinstance(value, str) and len(value) == 64 and
                     all(character in '0123456789abcdef'
                         for character in value)
                     for value in message_hashes),
                 'PLAN_PHASE_MESSAGE_HASHES')
        _require(phase.get('handle_exact_id') == HANDLE_EXACT_ID and
                 phase.get('handle_exact_retained') is True,
                 'PLAN_HANDLE_EXACT')
        _require(abs(float(phase.get('handle_angle_rad', math.inf)) -
                     HANDLE_ANGLE_RAD) <= 1e-12,
                 'PLAN_HANDLE_ANGLE')
        unhashed = dict(phase)
        stored_hash = unhashed.pop('phase_hash', None)
        _require(stored_hash == canonical_hash(unhashed), 'PLAN_PHASE_HASH')
    _require(plan.get('phase_hashes') ==
             [value['phase_hash'] for value in phases],
             'PLAN_PHASE_HASHES')
    schedule = plan.get('dynamic_object_schedule')
    _require(isinstance(schedule, list) and len(schedule) == len(phases),
             'PLAN_DYNAMIC_SCHEDULE')
    _require(plan.get('dynamic_object_schedule_hash') ==
             canonical_hash(schedule), 'PLAN_DYNAMIC_SCHEDULE_HASH')
    expected_schedule = [{
        'phase': phase['name'],
        'door_angle_start_rad': phase['door_angle_start_rad'],
        'door_angle_end_rad': phase['door_angle_end_rad'],
        'handle_angle_rad': phase['handle_angle_rad'],
        'handle_exact_id': phase['handle_exact_id'],
        'handle_exact_retained': phase['handle_exact_retained'],
        'door_support': phase['door_support'],
        'cube_support': phase['cube_support'],
    } for phase in phases]
    _require(schedule == expected_schedule, 'PLAN_DYNAMIC_SCHEDULE_CONTENT')
    for item, spec in zip(schedule, PHASE_SPECS):
        _require(item.get('handle_exact_id') == HANDLE_EXACT_ID and
                 item.get('handle_exact_retained') is True,
                 'PLAN_DYNAMIC_HANDLE')
        _require(abs(float(item.get('handle_angle_rad', math.inf)) -
                     HANDLE_ANGLE_RAD) <= 1e-12,
                 'PLAN_DYNAMIC_HANDLE_ANGLE')
        _require(abs(float(item.get('door_angle_start_rad', math.inf)) -
                     spec.door_start_rad) <= 1e-12 and
                 abs(float(item.get('door_angle_end_rad', math.inf)) -
                     spec.door_end_rad) <= 1e-12,
                 'PLAN_DYNAMIC_DOOR_ANGLE')
    if 'retime_audits' in plan or 'planned_action_duration_s' in plan:
        timing = plan.get('retime_audits')
        _require(isinstance(timing, list) and len(timing) == len(PHASE_SPECS),
                 'PLAN_RETIME_AUDITS')
        _require([value.get('phase') for value in timing] ==
                 list(PHASE_NAMES), 'PLAN_RETIME_PHASES')
        _require(all(value.get('status') == 'PASS' for value in timing),
                 'PLAN_RETIME_STATUS')
        duration = plan.get('planned_action_duration_s')
        _require(isinstance(duration, (int, float)) and
                 math.isfinite(duration) and
                 abs(duration - sum(value.get('retimed_duration_s', math.inf)
                                    for value in timing)) <= 1e-9 and
                 0.0 < duration <= MAXIMUM_ACTION_DURATION_S,
                 'PLAN_RETIME_DURATION')
        _require(plan.get('timing_audit_status') == 'PASS',
                 'PLAN_TIMING_STATUS')


def validate_exact_audit_evidence(evidence: Mapping, plan: Mapping) -> None:
    """Validate fail-closed evidence expected from the future exact auditor."""
    _require(isinstance(evidence, Mapping), 'AUDIT_TYPE')
    _require(evidence.get('schema') == AUDIT_EVIDENCE_SCHEMA,
             'AUDIT_SCHEMA')
    _require(evidence.get('auditor_version') == AUDITOR_VERSION,
             'AUDIT_VERSION')
    _require(evidence.get('audit_method') ==
             'FCL_UNPADDED_DYNAMIC_EXACT_STL', 'AUDIT_METHOD')
    _require(evidence.get('plan_hash') == canonical_hash(plan),
             'AUDIT_PLAN_HASH')
    _require(evidence.get('raw_scene_hash') == plan.get('raw_scene_hash'),
             'AUDIT_RAW_SCENE_HASH')
    _require(evidence.get('physical_scene_hash') ==
             plan.get('physical_scene_hash'), 'AUDIT_PHYSICAL_SCENE_HASH')
    _require(evidence.get('dynamic_object_schedule_hash') ==
             plan.get('dynamic_object_schedule_hash'),
             'AUDIT_DYNAMIC_SCHEDULE_HASH')
    _require(evidence.get('controller_commands_issued') == 0,
             'AUDIT_CONTROLLER_COMMANDS')
    _require(isinstance(evidence.get('samples'), int) and
             evidence['samples'] > 0, 'AUDIT_SAMPLES')
    _require(evidence.get('collision_free') is True, 'AUDIT_COLLISION')
    clearance = evidence.get('minimum_environment_clearance_m')
    _require(isinstance(clearance, (int, float)) and
             math.isfinite(clearance) and clearance >= REQUIRED_CLEARANCE_M,
             'AUDIT_CLEARANCE')
    _require(evidence.get('minimum_environment_clearance_is_lower_bound') is
             True, 'AUDIT_CLEARANCE_BOUND_KIND')
    derivation = evidence.get('environment_clearance_derivation')
    _require(isinstance(derivation, Mapping) and
             derivation.get('exact_world_objects') ==
             'direct_unpadded_FCL_distance' and
             derivation.get('environment_table_10mm_margin') ==
             '0.010_plus_guard_distance' and
             derivation.get('green_cube_10mm_guard') ==
             '0.010_plus_guard_distance' and
             derivation.get('self_pairs_excluded') is True,
             'AUDIT_CLEARANCE_DERIVATION')
    _require(evidence.get('forward_passed') is True, 'AUDIT_FORWARD')
    _require(evidence.get('reverse_passed') is True, 'AUDIT_REVERSE')
    _require(evidence.get('timing_passed') is True, 'AUDIT_TIMING')
    _require(evidence.get('no_partial') is True, 'AUDIT_PARTIAL')
    duration = evidence.get('planned_action_duration_s')
    _require(isinstance(duration, (int, float)) and
             0.0 < duration <= MAXIMUM_ACTION_DURATION_S,
             'AUDIT_DURATION')
    if 'planned_action_duration_s' in plan:
        _require(abs(float(duration) -
                     float(plan['planned_action_duration_s'])) <= 1e-9,
                 'AUDIT_PLAN_DURATION')

    handle = evidence.get('dynamic_handle_exact')
    _require(isinstance(handle, Mapping), 'AUDIT_HANDLE')
    _require(handle.get('id') == HANDLE_EXACT_ID, 'AUDIT_HANDLE_ID')
    _require(handle.get('geometry_type') == 'mesh' and
             handle.get('mesh_count') == 1 and
             handle.get('primitive_count') == 0,
             'AUDIT_HANDLE_GEOMETRY')
    _require(handle.get('geometry_sha256') == HANDLE_STL_SHA256,
             'AUDIT_HANDLE_STL_HASH')
    _require(handle.get('retained_every_phase') is True,
             'AUDIT_HANDLE_RETAINED')
    _require(handle.get('audited_phases') == list(PHASE_NAMES),
             'AUDIT_HANDLE_PHASES')
    _require(abs(float(handle.get('angle_rad', math.inf)) -
                 HANDLE_ANGLE_RAD) <= 1e-6,
             'AUDIT_HANDLE_ANGLE')
    _require(abs(float(handle.get('door_angle_min_rad', math.inf))) <=
             1e-6 and
             abs(float(handle.get('door_angle_max_rad', math.inf)) -
                 DOOR_OPEN_ANGLE_RAD) <= 1e-6,
             'AUDIT_HANDLE_DOOR_COVERAGE')
    _require(tuple(tuple(value) for value in evidence.get(
        'right_handle_allowed_contact_pairs', ())) ==
        RIGHT_HANDLE_ALLOWED_CONTACT_PAIRS,
        'AUDIT_RIGHT_HANDLE_ACM')
    _require(evidence.get('right_door_allowed_contact_pairs') == [],
             'AUDIT_RIGHT_DOOR_ACM')
    contact_window = evidence.get('right_handle_contact_window')
    _require(isinstance(contact_window, Mapping) and
             contact_window.get('door_open_message_range') == [5, 13] and
             contact_window.get('cube_message_range') == [0, 6] and
             contact_window.get('door_close_message_range') == [0, 8] and
             contact_window.get('post_release_retreat_strict') is True,
             'AUDIT_RIGHT_HANDLE_WINDOW')

    torso = evidence.get('torso_joint')
    _require(isinstance(torso, Mapping), 'AUDIT_TORSO')
    _require(torso.get('commanded') is False, 'AUDIT_TORSO_COMMAND')
    _require(abs(float(torso.get('minimum_rad', math.inf))) <= 1e-12 and
             abs(float(torso.get('maximum_rad', math.inf))) <= 1e-12,
             'AUDIT_TORSO_ZERO')

    left_handle = evidence.get('left_handle')
    _require(isinstance(left_handle, Mapping), 'AUDIT_LEFT_HANDLE')
    _require(left_handle.get('allowed_contact_pairs') == [],
             'AUDIT_LEFT_HANDLE_ACM')
    _require(left_handle.get('contact_count') == 0,
             'AUDIT_LEFT_HANDLE_CONTACT')
    left_clearance = left_handle.get('minimum_clearance_m')
    _require(isinstance(left_clearance, (int, float)) and
             math.isfinite(left_clearance) and
             left_clearance >= REQUIRED_CLEARANCE_M,
             'AUDIT_LEFT_HANDLE_CLEARANCE')

    projection = evidence.get('mask_projection')
    _require(isinstance(projection, Mapping), 'AUDIT_MASK_PROJECTION')
    _require(projection.get('unknown_visual_mask_ids') == [],
             'AUDIT_UNKNOWN_MASK')
    _require(projection.get('handle_exact_retained') is True,
             'AUDIT_MASK_REMOVED_HANDLE')
    _require(projection.get('raw_scene_hash') == plan['raw_scene_hash'] and
             projection.get('physical_scene_hash') ==
             plan['physical_scene_hash'], 'AUDIT_MASK_SCENE_HASH')
    removed = projection.get('removed_visual_mask_ids')
    _require(isinstance(removed, list) and
             len(removed) == len(set(removed)) and
             set(removed).issubset(set(PERCEPTION_ONLY_MASK_IDS)),
             'AUDIT_MASK_IDS')
    projection_classification = projection.get('classification')
    if removed:
        _require(set(MASK_CONFLICT_PAIR).issubset(set(removed)) and
                 projection_classification ==
                 'ALLOWLIST_MASKS_PROJECTED',
                 'AUDIT_MASK_PROJECTION_BRANCH')
        masks_projected = True
    else:
        _require(projection_classification ==
                 'RAW_SCENE_ALREADY_PHYSICAL',
                 'AUDIT_MASK_PROJECTION_BRANCH')
        masks_projected = False

    recheck = evidence.get('mask_conflict_recheck')
    _require(isinstance(recheck, Mapping), 'AUDIT_MASK_RECHECK')
    _require(tuple(recheck.get('visual_pair', ())) == MASK_CONFLICT_PAIR,
             'AUDIT_MASK_PAIR')
    _require(recheck.get('phase') == 'cube_release' and
             recheck.get('source_message_index') == 4 and
             recheck.get('dense_sample_index') == 527,
             'AUDIT_MASK_SAMPLE')
    _require(recheck.get('dense_index_scope') ==
             MASK_RECHECK_DENSE_SCOPE and
             abs(float(recheck.get('dense_grid_max_angular_step_rad',
                                   math.inf)) -
                 MASK_RECHECK_ANGULAR_STEP_RAD) <= 1e-12 and
             abs(float(recheck.get('dense_grid_max_prismatic_step_m',
                                   math.inf)) -
                 MASK_RECHECK_PRISMATIC_STEP_M) <= 1e-12,
             'AUDIT_MASK_DENSE_GRID')
    visual_distance = recheck.get('observed_visual_distance_m')
    if masks_projected:
        _require(isinstance(visual_distance, (int, float)) and
                 math.isfinite(visual_distance) and
                 abs(visual_distance - MASK_OBSERVED_DISTANCE_M) <= 1e-9,
                 'AUDIT_MASK_VISUAL_DISTANCE')
        _require(recheck.get('classification') ==
                 'MASK_ONLY_FALSE_POSITIVE',
                 'AUDIT_MASK_CLASSIFICATION')
    else:
        _require(visual_distance is None and
                 recheck.get('classification') ==
                 'RAW_SCENE_ALREADY_PHYSICAL',
                 'AUDIT_MASK_CLASSIFICATION')
    neighborhood = recheck.get('physical_exact_stl_neighborhood')
    _require(isinstance(neighborhood, list) and
             [item.get('offset') for item in neighborhood] == [-1, 0, 1] and
             [item.get('dense_sample_index') for item in neighborhood] ==
             [526, 527, 528],
             'AUDIT_MASK_NEIGHBORHOOD')
    for item in neighborhood:
        _require(item.get('exact_stl') is True,
                 'AUDIT_MASK_EXACT_STL')
        pair = item.get('pair')
        _require(isinstance(pair, list) and len(pair) == 2 and
                 pair[0] == HANDLE_EXACT_ID and
                 pair[1] in PHYSICAL_LEFT_FINGER_2_LINKS,
                 'AUDIT_MASK_PHYSICAL_PAIR')
        distance = item.get('distance_m')
        _require(isinstance(distance, (int, float)) and
                 math.isfinite(distance) and
                 distance >= REQUIRED_CLEARANCE_M,
                 'AUDIT_MASK_PHYSICAL_CLEARANCE')

    phase_metrics = evidence.get('per_phase_metrics')
    _require(isinstance(phase_metrics, list) and
             [item.get('phase') for item in phase_metrics] ==
             list(PHASE_NAMES), 'AUDIT_PHASE_METRICS')
    for item in phase_metrics:
        samples = item.get('samples')
        authorized = item.get('right_handle_authorized_samples')
        phase_clearance = item.get('minimum_environment_clearance_m')
        _require(isinstance(samples, int) and samples > 0 and
                 isinstance(authorized, int) and
                 0 <= authorized <= samples and
                 item.get('minimum_is_lower_bound') is True and
                 isinstance(phase_clearance, (int, float)) and
                 math.isfinite(phase_clearance) and
                 phase_clearance >= REQUIRED_CLEARANCE_M,
                 'AUDIT_PHASE_METRIC_CONTENT')


def make_certificate(plan: Mapping, evidence: Mapping) -> dict:
    """Seal a valid certificate only after all exact evidence gates pass."""
    validate_middle_plan(plan)
    validate_exact_audit_evidence(evidence, plan)
    return {
        'schema': CERTIFICATE_SCHEMA,
        'worker_version': WORKER_VERSION,
        'topology': TOPOLOGY,
        'plan_hash': canonical_hash(plan),
        'audit_evidence_hash': canonical_hash(evidence),
        'physical_scene_hash': plan['physical_scene_hash'],
        'source_asset_hashes': copy.deepcopy(SOURCE_ASSET_HASHES),
        'seam_state_hash': SEAM_STATE_HASH,
        'minimum_environment_clearance_m':
            evidence['minimum_environment_clearance_m'],
        'planned_action_duration_s': evidence['planned_action_duration_s'],
        'left_handle_stage_count': 0,
        'torso_controller_authorized': False,
        'controller_commands_issued': 0,
        'valid': True,
        'status': 'PASS',
    }
