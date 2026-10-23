import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Point
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'runtime_evidence/centrifuge_48box_loading_20260923_r121'))
from plan_candidate import CandidatePlanner
from probe_loading_collision import message_pose, transform, mesh_vertices


def pose_matrix(value):
    matrix = np.eye(4)
    matrix[:3, 3] = [value.get('position', {}).get(axis, 0) for axis in 'xyz']
    matrix[:3, :3] = Rotation.from_quat([value['orientation'].get(axis, 0) for axis in 'xyzw']).as_matrix()
    return matrix


class ObservedPlanner(CandidatePlanner):
    def __init__(self, poses, robot_path=None):
        self.observed_poses = poses
        super().__init__(ROOT, robot_path or ROOT / 'runtime_evidence/centrifuge_robot_press_20260922_r119/robot.urdf',
                         ROOT / 'runtime_evidence/centrifuge_robot_press_20260922_r119/passive_leg_baseline.json')

    def check(self, joints, verbose=False):
        combined = {name: value for name, value in self.initial.items() if 'dex1_finger' in name}
        combined.update(joints)
        return super().check(combined, verbose)

    def add_scene(self, name, translation):
        model = ET.parse(self.description / 'models' / name / 'model.sdf').getroot().find('model')
        offset = np.eye(4)
        offset[:3, 3] = translation
        if name == 'centrifuge':
            offset = pose_matrix(self.observed_poses['centrifuge'])
        for link in model.findall('link'):
            link_pose = transform(link.findtext('pose', '0 0 0 0 0 0'))
            joint_rotation = np.eye(4)
            if name == 'centrifuge' and link.get('name') in self.observed_poses:
                link_pose = pose_matrix(self.observed_poses[link.get('name')])
            elif name == 'centrifuge' and link.get('name') == 'lid':
                joint_rotation[:3, :3] = Rotation.from_euler('y', 120, degrees=True).as_matrix()
            for collision in link.findall('collision'):
                geometry = collision.find('geometry')
                matrix = offset @ link_pose @ joint_rotation @ transform(collision.findtext('pose', '0 0 0 0 0 0'))
                item = CollisionObject()
                item.header.frame_id = 'world'
                item.id = name + '_' + link.get('name') + '_' + collision.get('name')
                item.operation = CollisionObject.ADD
                if geometry.find('mesh') is not None:
                    path = self.description / geometry.findtext('mesh/uri').split('h1_2_description/', 1)[1]
                    scale = np.array([float(value) for value in geometry.findtext('mesh/scale', '1 1 1').split()])
                    triangles = mesh_vertices(path) * scale
                    mesh = Mesh()
                    mesh.vertices = [Point(x=float(vertex[0]), y=float(vertex[1]), z=float(vertex[2]))
                                     for vertex in triangles.reshape(-1, 3)]
                    mesh.triangles = [MeshTriangle(vertex_indices=[index * 3, index * 3 + 1, index * 3 + 2])
                                      for index in range(len(triangles))]
                    item.meshes = [mesh]
                    item.mesh_poses = [message_pose(matrix)]
                else:
                    primitive = SolidPrimitive()
                    if geometry.find('box') is not None:
                        primitive.type = SolidPrimitive.BOX
                        primitive.dimensions = [float(value) for value in geometry.findtext('box/size').split()]
                    else:
                        primitive.type = SolidPrimitive.CYLINDER
                        primitive.dimensions = [float(geometry.findtext('cylinder/length')),
                                                float(geometry.findtext('cylinder/radius'))]
                    item.primitives = [primitive]
                    item.primitive_poses = [message_pose(matrix)]
                self.scene.apply_collision_object(item)


def make_alignment_scene(positions, poses, robot_path=None):
    planner = ObservedPlanner(poses, robot_path)
    planner.initial = dict(positions)
    planner.state.joint_positions = planner.initial
    planner.state.update()
    planner.update_scene_state()
    for side in ('left', 'right'):
        item = CollisionObject()
        item.id = 'box48_' + side
        item.header.frame_id = 'world'
        item.operation = CollisionObject.ADD
        item.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.1187450051085, .08814010043, .0595])]
        item.primitive_poses = [message_pose(pose_matrix(poses[item.id]))]
        planner.scene.apply_collision_object(item)
    return planner

