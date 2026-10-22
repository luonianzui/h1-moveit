"""Build the isolated Unitree H1_2 Gazebo navigation URDF."""

import argparse
from copy import deepcopy
import subprocess
import sys
import xml.etree.ElementTree as ET

from .locomotion_contract import EXPECTED_CONTROLLED_JOINTS
from .locomotion_contract import EXPECTED_LEG_JOINTS
from .navigation_urdf_adapter import adapt_urdf
from .unitree_h12_contract import load_contract


def _parse_arguments(arguments):
    parser = argparse.ArgumentParser()
    parser.add_argument('--source-xacro', required=True)
    parser.add_argument('--contract-file', required=True)
    parser.add_argument('--controller-parameters-file', required=True)
    return parser.parse_args(arguments)


def _freeze_official_upper_body(root):
    leg_joints = set(EXPECTED_LEG_JOINTS)
    upper_body_joints = set(EXPECTED_CONTROLLED_JOINTS) - leg_joints
    joints = {joint.get('name'): joint for joint in root.findall('joint')}
    if not upper_body_joints.issubset(joints):
        missing = sorted(upper_body_joints - set(joints))
        raise ValueError(f'upper-body joints missing: {missing}')
    for joint_name in upper_body_joints:
        joint = joints[joint_name]
        joint.set('type', 'fixed')
        for child_name in ('axis', 'limit', 'dynamics', 'mimic', 'safety_controller'):
            child = joint.find(child_name)
            if child is not None:
                joint.remove(child)
    control = root.find("ros2_control[@name='H1_2NavigationGazeboSystem']")
    if control is None:
        raise ValueError('navigation ros2_control block is missing')
    for joint in tuple(control.findall('joint')):
        if joint.get('name') not in leg_joints:
            control.remove(joint)


def _replace_foot_collisions(root):
    for side in ('left', 'right'):
        link_name = f'{side}_ankle_roll_link'
        link = root.find(f"link[@name='{link_name}']")
        if link is None:
            raise ValueError(f'foot link missing: {link_name}')
        visual = link.find('visual')
        geometry = visual.find('geometry') if visual is not None else None
        mesh = geometry.find('mesh') if geometry is not None else None
        if mesh is None or not mesh.get('filename', '').endswith(
                f'{link_name}.STL'):
            raise ValueError(f'official foot visual mesh missing: {link_name}')
        collisions = tuple(link.findall('collision'))
        if len(collisions) != 1:
            raise ValueError(f'unexpected foot collision count: {link_name}')
        collision_name = collisions[0].get('name')
        link.remove(collisions[0])
        collision = ET.SubElement(
            link, 'collision', {'name': collision_name})
        ET.SubElement(collision, 'origin', {
            'xyz': '0 0 0',
            'rpy': '0 0 0',
        })
        collision.append(deepcopy(geometry))


def adapt_unitree_h12_urdf(urdf_text, contract):
    """Match the official 12-DOF rigid-upper-body collision model."""
    root = ET.fromstring(adapt_urdf(urdf_text, contract))
    _freeze_official_upper_body(root)
    _replace_foot_collisions(root)
    return ET.tostring(root, encoding='unicode')


def main(arguments=None):
    options = _parse_arguments(arguments)
    process = subprocess.run(
        [
            'xacro', options.source_xacro,
            'controller_parameters_file:='
            + options.controller_parameters_file,
        ], check=True, stdout=subprocess.PIPE, text=True)
    contract = load_contract(options.contract_file)
    sys.stdout.write(adapt_unitree_h12_urdf(process.stdout, contract))


if __name__ == '__main__':
    main()
