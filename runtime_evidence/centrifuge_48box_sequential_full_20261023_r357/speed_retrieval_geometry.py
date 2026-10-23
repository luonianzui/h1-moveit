from sequential_planning import *
from sequential_geometry import dense
from speed_policy import PROFILE


class SpeedRetrievalPlanner(SinglePlanner):
    def check_edge(self, first, last, phase, table_endpoint=False, initial_support=None):
        for fraction, joints in dense(first, last):
            vector = np.array([joints[name] for name in self.names])
            if np.any(vector < self.lower - 1e-12) or np.any(vector > self.upper + 1e-12):
                raise RuntimeError('Original shrunken joint bound exceeded')
            if self.attached:
                self.planner.scene.allowed_collision_matrix.set_entry('box48_' + self.payload,
                    'table_link_surface', table_endpoint or initial_support == 'table' or
                    initial_support == 'table_start_only' and fraction == 0)
                self.planner.scene.allowed_collision_matrix.set_entry('box48_' + self.payload,
                    'centrifuge_bucket_' + self.payload + '_bucket_' + self.payload + '_collision',
                    initial_support == 'basket')
            self.checked += 1
            if self.planner.check(joints)['collision']:
                self.planner.check(joints, verbose=True)
                raise RuntimeError('Single-holder dense full-scene collision: ' + json.dumps(dict(phase=phase, fraction=fraction, joints=joints)))
            if self.attached:
                body = self.planner.state.get_global_link_transform(self.hand + '_wrist_yaw_link') @ self.reference
                tilt = math.degrees(math.acos(float(np.clip(body[2, 2], -1., 1.))))
                self.maximum_payload_tilt_deg = max(getattr(self, 'maximum_payload_tilt_deg', 0.), tilt)
                if body[2, 2] <= math.cos(math.radians(PROFILE['retrieval_commanded_tilt_deg'])):
                    raise RuntimeError('Explicit five-degree simulation retrieval tilt exceeded')


def retrieve_transport(request, reference):
    hint = hints_for('retrieval', request['hand'])
    planner = SpeedRetrievalPlanner(request, reference)
    seed = {name: request['positions'][name] for name in planner.names}
    body = pose_matrix(request['poses']['box48_' + request['payload']])
    planner.attach(body, seed)
    bucket = pose_matrix(request['poses']['centrifuge']) @ pose_matrix(request['poses']['bucket_' + request['payload']])
    rim = bucket_rim_bounds(EVIDENCE.parents[1] / 'src/h1_2_description/models/centrifuge/model.sdf')[request['payload']]
    local = np.linalg.inv(bucket) @ body
    vertices = np.array(list(itertools.product((-1., 1.), repeat=3))) * np.array(SIZE) / 2
    bottom = (vertices @ local[:3, :3].T + local[:3, 3])[:, 2].min()
    axis_exit = body.copy()
    axis_exit[:3, 3] += (-bucket[:3, 0] * local[0, 3] - bucket[:3, 1] * local[1, 3]
                        + bucket[:3, 2] * max(0., float(rim + .006 - bottom)))
    high = body.copy()
    high[2, 3] = 1.15975
    high[:3, :3] = Rotation.from_euler('z', math.atan2(body[1, 0], body[0, 0])).as_matrix()
    outboard = high.copy()
    outboard[1, 3] = .32 if request['hand'] == 'left' else -.32
    yaw = math.pi if abs(math.atan2(body[1, 0], body[0, 0])) > math.pi / 2 else 0.
    table = outboard.copy()
    table[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
    table[:3, 3] = [.44, .38 if request['hand'] == 'left' else -.38, 1.12475 if request['hand'] == 'right' else 1.14475]
    down = table.copy()
    down[2, 3] = .84475
    stages = []
    for index, (phase, target) in enumerate([('retrieve_axis_exit', axis_exit), ('retrieve_lift', high),
                                            ('retrieve_outboard', outboard), ('retrieve_transfer', table), ('retrieve_setdown', down)]):
        guides = hint['stages'][index]['samples'][1:]
        stage = planner.guided_stage(body, target, seed, phase, guides, table_end=index == 4,
                                     support_start='basket' if index == 0 else None)
        stages.append(stage)
        body, seed = target, stage['samples'][-1]
    return dict(result(planner, stages, reference), table_yaw_rad=yaw,
                commanded_tilt_limit_deg=PROFILE['retrieval_commanded_tilt_deg'],
                maximum_planned_payload_tilt_deg=planner.maximum_payload_tilt_deg,
                world_level_after_axis_exit_with_original_actual_yaw=True,
                original_two_degree_certificate_not_claimed=True)
