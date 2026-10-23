import hashlib
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np


def mesh_bounds(path):
    data = Path(path).read_bytes()
    if len(data) < 84:
        raise ValueError('Truncated binary STL')
    count = struct.unpack_from('<I', data, 80)[0]
    if count == 0 or len(data) != 84 + 50 * count:
        raise ValueError('Exact complete binary STL required')
    facets = np.frombuffer(data, offset=84, dtype=np.dtype([
        ('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')]))
    vertices = facets['vertices'].reshape(-1, 3).astype(float)
    if not np.isfinite(vertices).all():
        raise ValueError('Finite source vertices required')
    lower, upper = vertices.min(axis=0), vertices.max(axis=0)
    if not np.all(upper > lower):
        raise ValueError('Nondegenerate foot dimensions required')
    return lower, upper, dict(source=str(path), source_sha256=hashlib.sha256(data).hexdigest(), triangles=count)


def make_sole_boxes(content, description):
    root = ET.fromstring(content)
    bases = [joint for joint in root.findall('joint') if joint.find('parent') is not None and joint.find('parent').get('link') == 'world']
    if len(bases) != 1 or bases[0].get('type') != 'fixed' or bases[0].find('child').get('link') != 'pelvis':
        raise ValueError('Only fixed-pelvis manipulation scene is qualified')
    records = []
    for side in ('left', 'right'):
        name = side + '_ankle_roll_link'
        link = root.find(f'./link[@name="{name}"]')
        collisions = link.findall('collision')
        if len(collisions) != 1:
            raise ValueError('Exactly one original foot collision required')
        collision = collisions[0]
        origin = collision.find('origin')
        if origin.attrib != dict(rpy='0 0 0', xyz='0 0 0'):
            raise ValueError('Unexpected source foot collision origin')
        geometry = collision.find('geometry')
        mesh = geometry.find('mesh')
        expected = 'package://h1_2_description/meshes/' + name + '.STL'
        if mesh is None or mesh.attrib != dict(filename=expected) or len(geometry) != 1:
            raise ValueError('Exact original mesh geometry required')
        lower, upper, provenance = mesh_bounds(Path(description) / 'meshes' / (name + '.STL'))
        origin.set('xyz', ' '.join(format(value, '.17g') for value in (lower + upper) / 2))
        geometry.remove(mesh)
        ET.SubElement(geometry, 'box', size=' '.join(format(value, '.17g') for value in upper - lower))
        records.append(dict(link=name, lower=lower.tolist(), upper=upper.tolist(), **provenance))
    return ET.tostring(root, encoding='unicode'), records
