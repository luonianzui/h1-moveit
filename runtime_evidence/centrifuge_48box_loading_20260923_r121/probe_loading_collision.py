import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from moveit.core.collision_detection import CollisionResult

ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / 'src/h1_2_moveit_config/scripts'))
from centrifuge_press_plan import PressPlanner, transform, mesh_vertices


def message_pose(matrix):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = matrix[:3, 3].tolist()
    quaternion = Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
    return pose


class LoadingPlanner(PressPlanner):
    def add_scene(self, name, translation):
        model = ET.parse(self.description / 'models' / name / 'model.sdf').getroot().find('model')
        offset = np.eye(4)
        offset[:3, 3] = translation
        for link in model.findall('link'):
            link_pose = transform(link.findtext('pose', '0 0 0 0 0 0'))
            joint_rotation = np.eye(4)
            if name == 'centrifuge' and link.get('name') == 'lid':
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

    def payload(self, name, centre, yaw=0):
        item = CollisionObject()
        item.id = name
        item.header.frame_id = 'world'
        item.operation = CollisionObject.ADD
        item.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.1187450051085, .08814010043, .0595])]
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
        matrix[:3, 3] = centre
        item.primitive_poses = [message_pose(matrix)]
        self.scene.apply_collision_object(item)

    def check(self, joints, verbose=False):
        initial = dict(self.initial)
        initial.update(joints)
        for side in ('left', 'right'):
            for number in (1, 2):
                name = f'{side}_dex1_finger_{number}_joint'
                initial[name] = joints.get(name, -.02)
        self.state.joint_positions = initial
        self.state.update()
        self.request.verbose = verbose
        self.request.contacts = verbose
        self.request.distance = False
        self.request.max_contacts = 100
        result = CollisionResult()
        self.scene.check_collision(self.request, result, self.state)
        return dict(collision=result.collision, reported_distance=result.distance)


def main():
    planner = LoadingPlanner(ROOT, ROOT / 'runtime_evidence/centrifuge_robot_press_20260922_r119/robot.urdf',
                             ROOT / 'runtime_evidence/centrifuge_robot_press_20260922_r119/passive_leg_baseline.json')
    records = json.loads((EVIDENCE / 'ik_probe.json').read_text())
    for record in records:
        for side in ('left', 'right'):
            centre = [.44, .38 if side == 'left' else -.38, .84475]
            yaw = 0
            if record['phase'] == 'place' and record['side'] == side:
                centre = np.array(record['target']) - [0, 0, .045]
                yaw = .309270119
            planner.payload('box48_' + side, centre, yaw)
        record['collision_check'] = planner.check(record['joints'])
        print(record['side'], record['phase'], record['pitch'], record['flip'], record['collision_check'], flush=True)
    with (EVIDENCE / 'ik_collision_probe.json').open('x') as output:
        json.dump(records, output, indent=2)


if __name__ == '__main__':
    main()
