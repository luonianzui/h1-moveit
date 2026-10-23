from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from sole_box import mesh_bounds


def world_transform(robot, link, positions):
    parents = {joint.find('child').get('link'): joint for joint in robot.findall('joint')}
    if link == 'world':
        return np.eye(4)
    if link not in parents:
        raise ValueError('Required fixed-world rooted joint chain missing')
    joint = parents[link]
    origin = joint.find('origin')
    local = np.eye(4)
    local[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
    local[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    if joint.get('type') != 'fixed':
        if joint.get('type') not in ('revolute', 'continuous'):
            raise ValueError('Only revolute leg joints are qualified')
        angle = float(positions[joint.get('name')])
        axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
        if not np.isfinite(angle) or not np.isclose(np.linalg.norm(axis), 1):
            raise ValueError('Finite original position and unit axis required')
        local[:3, :3] = local[:3, :3] @ Rotation.from_rotvec(axis * angle).as_matrix()
    return world_transform(robot, joint.find('parent').get('link'), positions) @ local


def make_standing_soles(content, description, positions):
    robot = ET.fromstring(content)
    bases = [joint for joint in robot.findall('joint') if joint.find('parent').get('link') == 'world']
    if len(bases) != 1 or bases[0].get('type') != 'fixed' or bases[0].find('child').get('link') != 'pelvis':
        raise ValueError('Only existing fixed-pelvis scene supported')
    records = []
    for side in ('left', 'right'):
        name = side + '_ankle_roll_link'
        link = robot.find('./link[@name="' + name + '"]')
        collisions = link.findall('collision')
        if len(collisions) != 1:
            raise ValueError('Exactly one original foot collision required')
        collision = collisions[0]
        origin = collision.find('origin')
        if origin.attrib != dict(rpy='0 0 0', xyz='0 0 0'):
            raise ValueError('Original unmodified foot collision required')
        geometry = collision.find('geometry')
        mesh = geometry.find('mesh')
        if mesh is None or len(geometry) != 1 or mesh.attrib != dict(filename='package://h1_2_description/meshes/' + name + '.STL'):
            raise ValueError('Original foot mesh identity required')
        filename = Path(description) / 'meshes' / (name + '.STL')
        unused_lower, unused_upper, provenance = mesh_bounds(filename)
        data = filename.read_bytes()
        facets = np.frombuffer(data, offset=84, dtype=np.dtype([
            ('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')]))
        vertices = facets['vertices'].reshape(-1, 3).astype(float)
        world = world_transform(robot, name, positions)
        rotated = vertices @ world[:3, :3].T
        lower, upper = rotated.min(axis=0), rotated.max(axis=0)
        original_lowest_world_z = float(world[2, 3] + lower[2])
        if not -.05 < original_lowest_world_z < -.02:
            raise ValueError('Only the measured legacy ground-intersecting fixed stance is qualified')
        lower[2] = -world[2, 3]
        if upper[2] - lower[2] < .02:
            raise ValueError('Ground-trimmed stationary sole is too thin')
        center = world[:3, :3].T @ ((lower + upper) / 2)
        rpy = Rotation.from_matrix(world[:3, :3].T).as_euler('xyz')
        origin.set('xyz', ' '.join(format(value, '.17g') for value in center))
        origin.set('rpy', ' '.join(format(value, '.17g') for value in rpy))
        geometry.remove(mesh)
        ET.SubElement(geometry, 'box', size=' '.join(format(value, '.17g') for value in upper - lower))
        records.append(dict(link=name, original_target_world_transform=world.tolist(),
                            center=center.tolist(), rpy=rpy.tolist(), size=(upper-lower).tolist(),
                            all_original_vertices_contained=False, original_lowest_world_z=original_lowest_world_z,
                            collision_approximation='original_stance_OBB_trimmed_at_ground_not_for_walking',
                            target_lowest_world_z=float(world[2, 3] + lower[2]),
                            **provenance))
    return ET.tostring(robot, encoding='unicode'), records
