#!/usr/bin/python3
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml


def transform(values):
    values = [float(value) for value in values.split()]
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('xyz', values[3:]).as_matrix()
    matrix[:3, 3] = values[:3]
    return matrix


def mesh_vertices(path):
    records = np.frombuffer(path.read_bytes(), offset=84, dtype=np.dtype([
        ('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')]))
    return records['vertices'].astype(float)


class PressPlanner:
    def __init__(self, root, urdf, passive_state=None):
        from moveit.core.robot_model import RobotModel
        from moveit.core.robot_state import RobotState
        from moveit.core.planning_scene import PlanningScene
        from moveit.core.collision_detection import CollisionRequest
        self.root = Path(root)
        self.model = RobotModel(str(urdf), str(self.root / 'src/h1_2_moveit_config/config/h1_2.srdf'))
        self.state = RobotState(self.model)
        self.initial = yaml.safe_load((self.root / 'src/h1_2_moveit_config/config/initial_positions.yaml').read_text())['initial_positions']
        if passive_state is not None:
            measured = json.loads(Path(passive_state).read_text())
            assert measured['samples'] >= 20
            assert max(measured['range'].values()) < .01
            assert len(measured['positions']) == 12
            for name, value in measured['positions'].items():
                assert any(token in name for token in ('hip_', 'ankle_', 'knee_'))
                assert name in self.initial and math.isfinite(value) and abs(value) < .2
                self.initial[name] = value
        self.state.joint_positions = self.initial
        self.state.update()
        self.scene = PlanningScene(self.model)
        for mounted in ('livox_mid360_link', 'realsense_camera_link'):
            self.scene.allowed_collision_matrix.set_entry('torso_link', mounted, True)
        self.request = CollisionRequest()
        self.request.distance = True
        self.request.contacts = False
        self.urdf = ET.parse(urdf).getroot()
        self.description = self.root / 'src/h1_2_description'
        self.add_scene('table', [.6, 0, 0])
        self.add_scene('centrifuge', [.6, 0, .815])

    def add_scene(self, name, translation):
        from geometry_msgs.msg import Point, Pose
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
        model = ET.parse(self.description / 'models' / name / 'model.sdf').getroot().find('model')
        offset = np.eye(4)
        offset[:3, 3] = translation
        for link in model.findall('link'):
            if name == 'centrifuge' and link.get('name') in ('rotor', 'bucket_left', 'bucket_right'):
                continue
            link_pose = transform(link.findtext('pose', '0 0 0 0 0 0'))
            angles = [0, 30, 60, 90, 120] if link.get('name') == 'lid' else [0]
            for angle in angles:
                hinge_rotation = np.eye(4)
                hinge_rotation[:3, :3] = Rotation.from_euler('y', angle, degrees=True).as_matrix()
                for collision in link.findall('collision'):
                    geometry = collision.find('geometry')
                    matrix = offset @ link_pose @ hinge_rotation @ transform(collision.findtext('pose', '0 0 0 0 0 0'))
                    pose = Pose()
                    pose.position.x, pose.position.y, pose.position.z = matrix[:3, 3].tolist()
                    quaternion = Rotation.from_matrix(matrix[:3, :3]).as_quat()
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion.tolist()
                    item = CollisionObject()
                    item.header.frame_id = 'world'
                    item.id = name + '_' + link.get('name') + '_' + collision.get('name') + '_' + str(angle)
                    item.operation = CollisionObject.ADD
                    if geometry.find('mesh') is not None:
                        uri = geometry.findtext('mesh/uri')
                        path = self.description / uri.split('h1_2_description/', 1)[1]
                        scale = np.array([float(value) for value in geometry.findtext('mesh/scale', '1 1 1').split()])
                        triangles = mesh_vertices(path) * scale
                        mesh = Mesh()
                        mesh.vertices = [Point(x=float(vertex[0]), y=float(vertex[1]), z=float(vertex[2]))
                                         for vertex in triangles.reshape(-1, 3)]
                        mesh.triangles = [MeshTriangle(vertex_indices=[index * 3, index * 3 + 1, index * 3 + 2])
                                          for index in range(len(triangles))]
                        item.meshes = [mesh]
                        item.mesh_poses = [pose]
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
                        item.primitive_poses = [pose]
                    self.scene.apply_collision_object(item)

    def set_arm(self, side, positions):
        self.state.joint_positions = self.initial
        self.state.set_joint_group_positions(side + '_arm', list(positions))
        self.state.update()

    def collision(self, side, positions, verbose=False):
        from moveit.core.collision_detection import CollisionResult
        self.set_arm(side, positions)
        self.request.verbose = verbose
        result = CollisionResult()
        self.scene.check_collision(self.request, result, self.state)
        return result.collision, result.distance

    def solve(self, side, rotation, target, local_point, seed):
        group = self.model.get_joint_model_group(side + '_arm')
        names = group.active_joint_model_names
        limits = [self.urdf.find(f"./joint[@name='{name}']/limit") for name in names]
        lower = [float(limit.get('lower')) + .02 for limit in limits]
        upper = [float(limit.get('upper')) - .02 for limit in limits]
        finger = side + '_dex1_finger_' + ('1' if side == 'left' else '2') + '_link'

        def residual(positions):
            self.set_arm(side, positions)
            pose = self.state.get_global_link_transform(finger)
            wrist = self.state.get_global_link_transform(side + '_wrist_yaw_link')
            point = pose[:3, :3] @ local_point + pose[:3, 3]
            return np.r_[20 * (point - target),
                         Rotation.from_matrix(rotation @ wrist[:3, :3].T).as_rotvec(),
                         .001 * (positions - seed)]

        result = least_squares(residual, np.clip(seed, lower, upper), bounds=(lower, upper),
                               max_nfev=300, ftol=1e-11, xtol=1e-11, gtol=1e-11)
        error = residual(result.x)
        if np.linalg.norm(error[:3]) / 20 > .00005 or np.linalg.norm(error[3:6]) > .001:
            raise ValueError('IK residual')
        return result.x

    def generate(self):
        contract = json.loads((self.description / 'models/centrifuge/LID_MOTION.json').read_text())
        normal = np.array(contract['button_outward_normal_model'])
        plans = {}
        diagnostics = []
        for side, button in [('left', 'lid'), ('right', 'rotor')]:
            names = self.model.get_joint_model_group(side + '_arm').active_joint_model_names
            home = np.array([self.initial[name] for name in names])
            finger = side + '_dex1_finger_' + ('1' if side == 'left' else '2') + '_link'
            cap_id = 'centrifuge_assembly_' + button + '_button_collision_0'
            self.scene.allowed_collision_matrix.set_entry(cap_id, finger, True)
            surface = np.array(contract[button + '_button_model_m']) + [.6, 0, .815] + .005 * normal
            vertices = mesh_vertices(self.description / 'meshes/dex1_1' /
                                     ('Link1_2.STL' if side == 'left' else 'Link2_2.STL')).reshape(-1, 3)
            for yaw in [0, -.25, -.5, -.75, -1.0]:
                yaw = yaw if side == 'left' else -yaw
                rotation = Rotation.from_euler('xyz', [0, .205395302728, yaw]).as_matrix()
                finger_rotation = rotation @ Rotation.from_euler('z', -math.pi / 2).as_matrix()
                projections = vertices @ (finger_rotation.T @ -normal)
                local_point = vertices[projections > projections.max() - 1e-6].mean(axis=0)
                try:
                    pre = self.solve(side, rotation, surface + .04 * normal, local_point, home)
                    press = self.solve(side, rotation, surface - .0005 * normal, local_point, pre)
                    self.set_arm(side, home)
                    home_pose = self.state.get_global_link_transform(finger)
                    home_rotation = self.state.get_global_link_transform(side + '_wrist_yaw_link')[:3, :3]
                    home_point = home_pose[:3, :3] @ local_point + home_pose[:3, 3]
                    park = self.solve(side, home_rotation, home_point + [-.14, 0, .08], local_point, home)
                    approach = [home + fraction * (park - home) for fraction in np.linspace(0, 1, 81)]
                    approach += [park + fraction * (pre - park) for fraction in np.linspace(0, 1, 101)[1:]]
                    push = []
                    seed = pre
                    for distance in np.linspace(.04, -.0005, 42):
                        seed = self.solve(side, rotation, surface + distance * normal, local_point, seed)
                        push.append(seed.copy())
                    samples = approach + push
                    checks = [self.collision(side, sample) for sample in samples]
                    bad = [index for index, value in enumerate(checks) if value[0]]
                    diagnostics.append(dict(side=side, yaw=yaw, bad_samples=bad,
                                            min_distance_m=min(value[1] for value in checks)))
                    if bad:
                        self.collision(side, samples[bad[0]], verbose=True)
                        continue
                    if min(value[1] for value in checks) < .005:
                        raise ValueError('Non-target clearance below 5 mm')
                    plans[side] = dict(button=button, joints=list(names), home=home.tolist(),
                                       pre=pre.tolist(), press=push[-1].tolist(),
                                       approach=[value.tolist() for value in approach],
                                       push=[value.tolist() for value in push],
                                       contact_link=finger, local_contact_point_m=local_point.tolist(),
                                       surface_world_m=surface.tolist(), yaw=yaw,
                                       min_nonallowed_distance_m=min(value[1] for value in checks))
                    break
                except ValueError as error:
                    diagnostics.append(dict(side=side, yaw=yaw, error=str(error)))
            self.scene.allowed_collision_matrix.set_entry(cap_id, finger, False)
        return dict(status='PASS' if len(plans) == 2 else 'FAIL', arms=plans,
                    diagnostics=diagnostics, initial_positions=self.initial,
                    structural_exemptions=[['torso_link', 'livox_mid360_link'],
                                           ['torso_link', 'realsense_camera_link']],
                    contact_penetration_m=.0005, torso_and_grippers_fixed=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--passive-state', type=Path)
    arguments = parser.parse_args()
    assert not arguments.output.exists()
    planner = PressPlanner(arguments.root, arguments.urdf, arguments.passive_state)
    result = planner.generate()
    result['urdf_sha256'] = hashlib.sha256(arguments.urdf.read_bytes()).hexdigest()
    if arguments.passive_state is not None:
        result['passive_state_sha256'] = hashlib.sha256(arguments.passive_state.read_bytes()).hexdigest()
    result['source_hashes'] = {
        name: hashlib.sha256((arguments.root / name).read_bytes()).hexdigest() for name in [
            'src/h1_2_description/models/centrifuge/model.sdf',
            'src/h1_2_description/models/table/model.sdf',
            'src/h1_2_moveit_config/config/h1_2.srdf',
            'src/h1_2_moveit_config/config/initial_positions.yaml',
            'src/h1_2_moveit_config/scripts/centrifuge_press_plan.py']}
    with arguments.output.open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps({key: value for key, value in result.items() if key != 'arms'}))
    assert result['status'] == 'PASS'


if __name__ == '__main__':
    main()
