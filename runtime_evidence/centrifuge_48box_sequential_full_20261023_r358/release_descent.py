import math

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive

from alignment_scene import make_alignment_scene, message_pose
from execute_loading import GROUP_JOINTS, pose_matrix


MAXIMUM_JOINT_STEP = .002
LOWER_DISTANCE_M = .045


def plan_descent(request, distance=LOWER_DISTANCE_M):
    side = request['side']
    if side not in ('left', 'right') or not math.isfinite(distance) or not 0 < distance <= .055:
        raise ValueError('Invalid bounded release descent')
    planner = make_alignment_scene(request['positions'], request['poses'])
    actual = pose_matrix(request['poses']['box48_' + side])
    planner.state.joint_positions = request['positions']
    planner.state.update()
    reference = np.linalg.inv(planner.state.get_global_link_transform(side + '_wrist_yaw_link')) @ actual
    attached = AttachedCollisionObject()
    attached.link_name = side + '_wrist_yaw_link'
    attached.touch_links = [side + f'_dex1_finger_{number}_{suffix}' for number in (1, 2)
                            for suffix in ('link', 'pad_link')]
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
    samples, checked, last_safe = [], 0, 0.
    for offset in np.linspace(0, distance, max(2, math.ceil(distance / .001) + 1)):
        body = actual.copy()
        body[2, 3] -= offset
        wrist_target = body @ np.linalg.inv(reference)
        previous = vector.copy()

        def residual(values):
            configuration = dict(request['positions'])
            configuration.update(zip(names, values))
            planner.state.joint_positions = configuration
            planner.state.update()
            wrist = planner.state.get_global_link_transform(side + '_wrist_yaw_link')
            return np.r_[20 * (wrist[:3, 3] - wrist_target[:3, 3]),
                         Rotation.from_matrix(wrist_target[:3, :3] @ wrist[:3, :3].T).as_rotvec(),
                         .0001 * (values - previous)]

        solved = least_squares(residual, np.clip(vector, lower, upper), bounds=(lower, upper),
                               max_nfev=400, ftol=1e-11, xtol=1e-11, gtol=1e-11)
        error = residual(solved.x)
        if np.linalg.norm(error[:3]) / 20 > .00002 or np.linalg.norm(error[3:6]) > math.radians(.01):
            return dict(status='FAIL', reason='IK', failed_descent_m=float(offset), last_safe_descent_m=last_safe)
        vector = solved.x
        count = max(2, math.ceil(float(np.max(np.abs(vector - previous))) / MAXIMUM_JOINT_STEP) + 1)
        for intermediate in np.linspace(previous, vector, count):
            if planner.check(dict(zip(names, intermediate)))['collision']:
                details = planner.check(dict(zip(names, intermediate)), verbose=True)
                return dict(status='FAIL', reason='FCL_loaded_descent', collision=details,
                            failed_descent_m=float(offset), last_safe_descent_m=last_safe)
            checked += 1
        samples.append(dict(zip(names, vector.tolist())))
        last_safe = float(offset)
    for fraction in np.linspace(0, 1, 101):
        configuration = dict(samples[-1])
        for name in GROUP_JOINTS[side + '_gripper']:
            configuration[name] = request['positions'][name] + fraction * (-.0199 - request['positions'][name])
        if planner.check(configuration)['collision']:
            return dict(status='FAIL', reason='FCL_gripper_release', fraction=float(fraction),
                        collision=planner.check(configuration, verbose=True), last_safe_descent_m=last_safe)
        checked += 1
    return dict(status='PASS', side=side, samples=samples, observed_grip_transform=reference.tolist(),
                initial_body=actual.tolist(), target_body=body.tolist(), descent_m=distance,
                checked_samples=checked, maximum_joint_step_rad=MAXIMUM_JOINT_STEP,
                planning_only_attachment=True, physical_attachment=False,
                gripper_opening_dense_checked=True, original_containment_gate_unchanged=True)
