"""Build the Gazebo navigation URDF with training actuator dynamics."""

import argparse
import math
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

from .locomotion_contract import load_contract


def _rotation_matrix(roll, pitch, yaw):
    cosine_roll, sine_roll = math.cos(roll), math.sin(roll)
    cosine_pitch, sine_pitch = math.cos(pitch), math.sin(pitch)
    cosine_yaw, sine_yaw = math.cos(yaw), math.sin(yaw)
    return np.asarray([
        [
            cosine_yaw * cosine_pitch,
            cosine_yaw * sine_pitch * sine_roll
            - sine_yaw * cosine_roll,
            cosine_yaw * sine_pitch * cosine_roll
            + sine_yaw * sine_roll,
        ],
        [
            sine_yaw * cosine_pitch,
            sine_yaw * sine_pitch * sine_roll
            + cosine_yaw * cosine_roll,
            sine_yaw * sine_pitch * cosine_roll
            - cosine_yaw * sine_roll,
        ],
        [
            -sine_pitch,
            cosine_pitch * sine_roll,
            cosine_pitch * cosine_roll,
        ],
    ], dtype=np.float64)


def _vector_attribute(element, name, default):
    return np.asarray([
        float(value) for value in element.get(name, default).split()
    ], dtype=np.float64)


def _add_reflected_armature(link, axis, armature):
    inertial = link.find('inertial')
    if inertial is None:
        raise ValueError(f"link {link.get('name')} has no inertial")
    inertia = inertial.find('inertia')
    if inertia is None:
        raise ValueError(f"link {link.get('name')} has no inertia tensor")
    origin = inertial.find('origin')
    rotation = np.eye(3, dtype=np.float64)
    if origin is not None:
        roll, pitch, yaw = _vector_attribute(origin, 'rpy', '0 0 0')
        rotation = _rotation_matrix(roll, pitch, yaw)
    axis_inertial = rotation.T @ axis
    axis_inertial /= np.linalg.norm(axis_inertial)
    tensor = np.asarray([
        [float(inertia.get('ixx')), float(inertia.get('ixy')),
         float(inertia.get('ixz'))],
        [float(inertia.get('ixy')), float(inertia.get('iyy')),
         float(inertia.get('iyz'))],
        [float(inertia.get('ixz')), float(inertia.get('iyz')),
         float(inertia.get('izz'))],
    ], dtype=np.float64)
    tensor += 0.5 * armature * (
        np.eye(3, dtype=np.float64)
        + np.outer(axis_inertial, axis_inertial))
    for name, value in (
            ('ixx', tensor[0, 0]), ('ixy', tensor[0, 1]),
            ('ixz', tensor[0, 2]), ('iyy', tensor[1, 1]),
            ('iyz', tensor[1, 2]), ('izz', tensor[2, 2])):
        inertia.set(name, format(value, '.17g'))


def adapt_urdf(urdf_text, contract):
    root = ET.fromstring(urdf_text)
    actuator = contract['training_actuator']
    joints = {joint.get('name'): joint for joint in root.findall('joint')}
    links = {link.get('name'): link for link in root.findall('link')}
    armature = (
        float(actuator['armature'])
        * float(contract['gazebo_runtime']['armature_scale']))
    damping = float(actuator['viscous_friction'])
    friction = float(actuator['friction'])
    for joint_name in actuator['controlled_joints']:
        joint = joints.get(joint_name)
        if joint is None:
            raise ValueError(f'controlled joint missing: {joint_name}')
        child = joint.find('child')
        axis_element = joint.find('axis')
        if child is None or axis_element is None:
            raise ValueError(f'controlled joint is incomplete: {joint_name}')
        link = links.get(child.get('link'))
        if link is None:
            raise ValueError(f'controlled child link missing: {joint_name}')
        axis = _vector_attribute(axis_element, 'xyz', '1 0 0')
        if not np.isfinite(axis).all() or np.linalg.norm(axis) < 1.0e-9:
            raise ValueError(f'controlled joint axis invalid: {joint_name}')
        _add_reflected_armature(link, axis, armature)
        dynamics = joint.find('dynamics')
        if dynamics is None:
            dynamics = ET.SubElement(joint, 'dynamics')
        dynamics.set('damping', format(damping, '.17g'))
        dynamics.set('friction', format(friction, '.17g'))
    return ET.tostring(root, encoding='unicode')


def _parse_arguments(arguments):
    parser = argparse.ArgumentParser()
    parser.add_argument('--source-xacro', required=True)
    parser.add_argument('--contract-file', required=True)
    return parser.parse_args(arguments)


def main(arguments=None):
    options = _parse_arguments(arguments)
    process = subprocess.run(
        ['xacro', options.source_xacro], check=True,
        stdout=subprocess.PIPE, text=True)
    contract = load_contract(options.contract_file)
    sys.stdout.write(adapt_urdf(process.stdout, contract))


if __name__ == '__main__':
    main()
