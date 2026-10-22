import copy
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as element_tree

import yaml
from ament_index_python.packages import get_package_share_directory
from moveit.core.collision_detection import CollisionRequest
from moveit.core.planning_scene import PlanningScene
from moveit.core.robot_model import RobotModel
from moveit.core.robot_state import RobotState
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from alignment_scene import ROOT, ObservedPlanner, message_pose, pose_matrix


def signature(path):
    status = path.stat()
    return (status.st_dev, status.st_ino, status.st_size, status.st_mtime_ns, status.st_ctime_ns)


class StaticResources:
    def __init__(self, robot_path):
        self.robot_path = Path(robot_path).resolve(strict=True)
        self.semantic_path = ROOT / 'src/h1_2_moveit_config/config/h1_2.srdf'
        self.initial_path = ROOT / 'src/h1_2_moveit_config/config/initial_positions.yaml'
        self.passive_path = ROOT / 'runtime_evidence/centrifuge_robot_press_20260922_r119/passive_leg_baseline.json'
        self.urdf = element_tree.parse(self.robot_path).getroot()
        self.dependencies = {}
        paths = {self.robot_path, self.semantic_path, self.initial_path, self.passive_path}
        for mesh in self.urdf.findall('.//mesh'):
            uri = mesh.get('filename')
            if uri.startswith('package://h1_2_description/'):
                paths.add(ROOT / 'src/h1_2_description' / uri.split('h1_2_description/', 1)[1])
            elif uri.startswith('package://'):
                package, relative = uri[len('package://'):].split('/', 1)
                paths.add(Path(get_package_share_directory(package)) / relative)
            elif uri.startswith('file://'):
                paths.add(Path(uri[7:]))
            elif uri.startswith('/'):
                paths.add(Path(uri))
            else:
                raise ValueError('Unbound robot geometry resource: ' + uri)
        for path in sorted(paths):
            path = path.resolve(strict=True)
            self.dependencies[str(path)] = dict(signature=signature(path),
                sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        self.model = RobotModel(str(self.robot_path), str(self.semantic_path))
        self.initial = yaml.safe_load(self.initial_path.read_text())['initial_positions']
        measured = json.loads(self.passive_path.read_text())
        assert measured['samples'] >= 20 and max(measured['range'].values()) < .01
        assert len(measured['positions']) == 12
        for name, value in measured['positions'].items():
            assert any(token in name for token in ('hip_', 'ankle_', 'knee_'))
            assert name in self.initial and math.isfinite(value) and abs(value) < .2
            self.initial[name] = value
        self.verify()

    def verify(self):
        for name, expected in self.dependencies.items():
            if signature(Path(name)) != tuple(expected['signature']):
                raise RuntimeError('Static planner resource changed; refusing stale model: ' + name)


class ResourceCache:
    def __init__(self):
        self.resources = None
        self.hits = 0
        self.builds = 0
        self.scenes = 0
        self.build_wall_s = 0.

    def get(self, robot_path):
        path = Path(robot_path).resolve(strict=True)
        if self.resources is None:
            began = time.monotonic()
            self.resources = StaticResources(path)
            self.build_wall_s += time.monotonic() - began
            self.builds += 1
        elif path != self.resources.robot_path:
            raise ValueError('A worker cannot silently mix robot resource identities')
        else:
            self.hits += 1
        self.resources.verify()
        return self.resources

    def snapshot(self):
        return dict(model_builds=self.builds, model_reuses=self.hits, fresh_scenes=self.scenes,
            model_build_wall_s=self.build_wall_s,
            model_dependencies={} if self.resources is None else self.resources.dependencies,
            shared_mutable_scene_or_robot_state=False, cached_paths_or_holder_references=False)


CACHE = ResourceCache()


class ResourceObservedPlanner(ObservedPlanner):
    def __init__(self, poses, robot_path):
        resources = CACHE.get(robot_path)
        self.observed_poses = copy.deepcopy(poses)
        self.root = ROOT
        self.model = resources.model
        self.state = RobotState(self.model)
        self.initial = dict(resources.initial)
        self.state.joint_positions = self.initial
        self.state.update()
        self.scene = PlanningScene(self.model)
        for mounted in ('livox_mid360_link', 'realsense_camera_link'):
            self.scene.allowed_collision_matrix.set_entry('torso_link', mounted, True)
        self.request = CollisionRequest()
        self.request.distance = True
        self.request.contacts = False
        self.urdf = copy.deepcopy(resources.urdf)
        self.description = ROOT / 'src/h1_2_description'
        self.add_scene('table', [.6, 0, 0])
        self.add_scene('centrifuge', [.6, 0, .815])
        CACHE.scenes += 1


def make_resource_scene(positions, poses, robot_path=None):
    if robot_path is None:
        raise ValueError('An explicit isolated robot identity is required')
    planner = ResourceObservedPlanner(poses, robot_path)
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


def install_worker_resources():
    import sequential_geometry
    sequential_geometry.make_alignment_scene = make_resource_scene
