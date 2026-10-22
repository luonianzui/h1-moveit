import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive

from alignment_scene import make_alignment_scene, message_pose
from execute_loading import GROUP_JOINTS


request = json.loads(Path(sys.argv[1]).read_text())
side = request['side']
assert side in ('left', 'right')
planner = make_alignment_scene(request['positions'], request['poses'])
actual = np.array(request['actual'])
reference = np.array(request['reference'])
target = np.eye(4)
target[:3, :3] = Rotation.from_euler('z', .309270119).as_matrix()
target[:3, 3] = [.603178339 if side == 'left' else .657963763,
                .085730038 if side == 'left' else -.085730037, 1.12025]
if np.linalg.norm(actual[:3, 3] - target[:3, 3]) > .005 or Rotation.from_matrix(actual[:3, :3].T @ target[:3, :3]).magnitude() > math.radians(3):
    raise RuntimeError('Release correction exceeds bounded 5 mm / 3 degree window')
attached = AttachedCollisionObject()
attached.link_name = side + '_wrist_yaw_link'
attached.touch_links = [side + f'_dex1_finger_{number}_{suffix}' for number in (1, 2) for suffix in ('link', 'pad_link')]
attached.object.id = 'box48_' + side
attached.object.header.frame_id = 'world'
attached.object.operation = CollisionObject.ADD
attached.object.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.1187450051085, .08814010043, .0595])]
attached.object.primitive_poses = [message_pose(actual)]
planner.update_scene_state()
planner.scene.process_attached_collision_object(attached)
planner.state = planner.scene.current_state
names = ['torso_joint'] + GROUP_JOINTS[side + '_arm']
limits = [planner.urdf.find(f"./joint[@name='{name}']/limit") for name in names]
lower = np.array([float(limit.get('lower')) + .02 for limit in limits])
upper = np.array([float(limit.get('upper')) - .02 for limit in limits])
lower[0], upper[0] = np.deg2rad([-90, 90])
vector = np.array([request['positions'][name] for name in names])
rotations = Slerp([0, 1], Rotation.from_matrix(np.array([actual[:3, :3], target[:3, :3]])))
path = []
checked = 0
for fraction in np.linspace(0, 1, 41):
    body = np.eye(4)
    body[:3, :3] = rotations(float(fraction)).as_matrix()
    body[:3, 3] = actual[:3, 3] + fraction * (target[:3, 3] - actual[:3, 3])
    wrist_target = body @ np.linalg.inv(reference)
    previous = vector.copy()

    def residual(positions):
        configuration = dict(planner.initial)
        configuration.update(zip(names, positions))
        planner.state.joint_positions = configuration
        planner.state.update()
        wrist = planner.state.get_global_link_transform(side + '_wrist_yaw_link')
        return np.r_[20 * (wrist[:3, 3] - wrist_target[:3, 3]),
                     Rotation.from_matrix(wrist_target[:3, :3] @ wrist[:3, :3].T).as_rotvec(),
                     .0001 * (positions - previous)]

    solved = least_squares(residual, np.clip(vector, lower, upper), bounds=(lower, upper),
                           max_nfev=400, ftol=1e-11, xtol=1e-11, gtol=1e-11)
    error = residual(solved.x)
    if np.linalg.norm(error[:3]) / 20 > .00002 or np.linalg.norm(error[3:6]) > math.radians(.01):
        raise RuntimeError('Bounded release correction IK failed')
    vector = solved.x
    count = max(2, int(np.ceil(np.max(np.abs(vector - previous)) / .002)) + 1)
    for intermediate in np.linspace(previous, vector, count):
        if planner.check(dict(zip(names, intermediate)))['collision']:
            raise RuntimeError('Bounded release correction collision')
        checked += 1
    path.append(dict(zip(names, vector.tolist())))
plan = dict(status='PASS', side=side, actual=actual.tolist(), target=target.tolist(),
            observed_grip_transform=reference.tolist(), checked_samples=checked,
            maximum_joint_step_rad=.002, planning_only_attachment=True,
            simulation_ground_truth_not_vision=True, samples=path)
with Path(sys.argv[2]).open('x') as stream:
    json.dump(plan, stream, indent=2)
print('PASS', checked)

