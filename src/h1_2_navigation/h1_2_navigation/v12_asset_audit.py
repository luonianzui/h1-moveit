"""Read-only asset parity audit for the frozen V12 locomotion policy."""

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_share_directory
import yaml

from .locomotion_contract import EXPECTED_LEG_JOINTS
from .locomotion_contract import file_identity
from .locomotion_contract import load_contract


JOINT_MATCH_FIELDS = (
    'type', 'parent', 'child', 'origin_xyz', 'origin_rpy',
    'axis', 'lower', 'upper', 'effort',
)


def _float_tuple(element, attribute):
    return tuple(float(value) for value in element.attrib[attribute].split())


def _parse_urdf(path):
    return ElementTree.parse(path).getroot()


def _joint_contract(root):
    result = {}
    for joint in root.findall('joint'):
        name = joint.attrib.get('name')
        if name not in EXPECTED_LEG_JOINTS:
            continue
        limit = joint.find('limit')
        result[name] = {
            'type': joint.attrib['type'],
            'parent': joint.find('parent').attrib['link'],
            'child': joint.find('child').attrib['link'],
            'origin_xyz': _float_tuple(joint.find('origin'), 'xyz'),
            'origin_rpy': _float_tuple(joint.find('origin'), 'rpy'),
            'axis': _float_tuple(joint.find('axis'), 'xyz'),
            'lower': float(limit.attrib['lower']),
            'upper': float(limit.attrib['upper']),
            'effort': float(limit.attrib['effort']),
            'velocity': float(limit.attrib['velocity']),
        }
    return result


def _inertial_contract(root):
    result = {}
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        inertia = inertial.find('inertia')
        result[link.attrib['name']] = {
            'mass': float(inertial.find('mass').attrib['value']),
            'origin_xyz': _float_tuple(inertial.find('origin'), 'xyz'),
            'origin_rpy': _float_tuple(inertial.find('origin'), 'rpy'),
            'inertia': tuple(float(inertia.attrib[name]) for name in (
                'ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')),
        }
    return result


def _file_checks(contract):
    checks = {}
    specifications = {'checkpoint': contract['policy']['checkpoint']}
    specifications.update({
        name: specification
        for name, specification in contract['assets'].items()
        if isinstance(specification, dict) and 'path' in specification
    })
    for name, specification in specifications.items():
        path = Path(specification['path'])
        actual = file_identity(path)
        checks[name] = {
            'path': str(path),
            'size': actual['size'],
            'size_match': (
                'size' not in specification
                or actual['size'] == specification['size']),
            'sha256': actual['sha256'],
            'sha256_match': actual['sha256'] == specification['sha256'],
        }
    return checks


def audit_assets(contract):
    """Return matched properties and explicit deployment blockers."""
    file_checks = _file_checks(contract)
    assets = contract['assets']
    snapshot_text = Path(assets['isaac_env_snapshot']['path']).read_text(
        encoding='utf-8')
    snapshot_claims = {
        'task_source_sha256_match': (
            f"task_source_sha256: {assets['task_source_sha256']}"
            in snapshot_text),
        'robot_asset_sha256_match': (
            f"robot_asset_sha256: {assets['isaac_usd']['sha256']}"
            in snapshot_text),
    }
    isaac_root = _parse_urdf(assets['isaac_source_urdf']['path'])
    gazebo_root = _parse_urdf(assets['gazebo_model_urdf']['path'])
    isaac_joints = _joint_contract(isaac_root)
    gazebo_joints = _joint_contract(gazebo_root)
    matched_joint_fields = {}
    velocity_matches = {}
    for name in EXPECTED_LEG_JOINTS:
        matched_joint_fields[name] = all(
            isaac_joints[name][field] == gazebo_joints[name][field]
            for field in JOINT_MATCH_FIELDS
        )
        velocity_matches[name] = (
            isaac_joints[name]['velocity'] == gazebo_joints[name]['velocity'])

    isaac_inertials = _inertial_contract(isaac_root)
    gazebo_inertials = _inertial_contract(gazebo_root)
    common_inertials = sorted(set(isaac_inertials) & set(gazebo_inertials))
    inertial_matches = {
        name: isaac_inertials[name] == gazebo_inertials[name]
        for name in common_inertials
    }
    inertial_mismatches = [
        name for name, matched in inertial_matches.items() if not matched
    ]
    collision_counts = {
        'isaac_source': sum(
            len(link.findall('collision')) for link in isaac_root.findall('link')),
        'gazebo_source': sum(
            len(link.findall('collision')) for link in gazebo_root.findall('link')),
    }

    with Path(assets['gazebo_initial_positions']['path']).open(
            encoding='utf-8') as stream:
        gazebo_positions = yaml.safe_load(stream)['initial_positions']
    training_positions = contract['training_default_positions']
    default_position_matches = {
        name: gazebo_positions[name] == training_positions[name]
        for name in EXPECTED_LEG_JOINTS
    }

    spawn_text = Path(assets['gazebo_spawn_xacro']['path']).read_text(
        encoding='utf-8')
    controller_text = Path(assets['gazebo_controllers']['path']).read_text(
        encoding='utf-8')
    export_paths = [Path(path) for path in assets['expected_exports']]
    gates = {
        'identity': all(
            result['size_match'] and result['sha256_match']
            for result in file_checks.values()) and all(
                snapshot_claims.values()),
        'joint_kinematics_limits_effort': all(matched_joint_fields.values()),
        'joint_velocity_limits': all(velocity_matches.values()),
        'inertials': all(inertial_matches.values()),
        'collision_geometry': collision_counts['isaac_source'] ==
        collision_counts['gazebo_source'],
        'default_leg_pose': all(default_position_matches.values()),
        'free_root': 'name="world_joint" type="fixed"' not in spawn_text,
        'leg_controller': 'leg_controller' in controller_text,
        'imu_sensor': 'type="imu"' in spawn_text,
        'foot_contact_sensors': spawn_text.count('type="contact"') >= 2,
        'policy_export': all(path.is_file() for path in export_paths),
    }
    blockers = [name for name, passed in gates.items() if not passed]
    return {
        'baseline_valid': gates['identity'],
        'deployment_ready': not blockers,
        'gates': gates,
        'blockers': blockers,
        'file_checks': file_checks,
        'snapshot_claims': snapshot_claims,
        'matched_joint_count': sum(matched_joint_fields.values()),
        'velocity_limit_match_count': sum(velocity_matches.values()),
        'matched_inertial_count': sum(inertial_matches.values()),
        'common_inertial_count': len(common_inertials),
        'inertial_mismatches': inertial_mismatches,
        'total_mass': {
            'isaac_source': sum(
                value['mass'] for value in isaac_inertials.values()),
            'gazebo_source': sum(
                value['mass'] for value in gazebo_inertials.values()),
        },
        'collision_counts': collision_counts,
        'default_position_matches': default_position_matches,
        'expected_exports_exist': {
            str(path): path.is_file() for path in export_paths
        },
    }


def main(args=None):
    """Run the read-only audit and fail only on identity drift."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--contract')
    parsed = parser.parse_args(args)
    contract_path = parsed.contract
    if contract_path is None:
        contract_path = Path(get_package_share_directory(
            'h1_2_navigation')) / 'config' / 'v12_locomotion.yaml'
    report = audit_assets(load_contract(contract_path))
    print(json.dumps(report, indent=2, sort_keys=True))
    if not report['baseline_valid']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
