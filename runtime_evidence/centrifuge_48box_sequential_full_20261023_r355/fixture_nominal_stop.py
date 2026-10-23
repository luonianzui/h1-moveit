import hashlib
import json
from pathlib import Path


EVIDENCE = Path(__file__).resolve().parent


def main():
    original = EVIDENCE / 'offline_dual_fixture.py'
    source = original.read_text()
    marker = "    stopped.update(mode='retrieval_pickup', assignment=dict(right=order[0], left=order[1]), retrieval=True)"
    assert source.count(marker) == 1
    replacement = """    from scipy.spatial.transform import Rotation
    rotor = pose_matrix(stopped['poses']['rotor'])
    rotor_yaw = np.arctan2(rotor[1, 0], rotor[0, 0])
    correction = Rotation.from_euler('z', -rotor_yaw).as_matrix()
    local_centre = rotor[:3, 3]
    world_centre = (pose_matrix(stopped['poses']['centrifuge']) @ np.r_[local_centre, 1.])[:3]
    for payload in ('left', 'right'):
        for key, centre in (('bucket_' + payload, local_centre), ('box48_' + payload, world_centre)):
            body = pose_matrix(stopped['poses'][key])
            body[:3, 3] = centre + correction @ (body[:3, 3] - centre)
            body[:3, :3] = correction @ body[:3, :3]
            stopped['poses'][key]['position'] = dict(zip('xyz', body[:3, 3].tolist()))
            stopped['poses'][key]['orientation'] = dict(zip('xyzw', Rotation.from_matrix(body[:3, :3]).as_quat().tolist()))
    stopped['poses']['rotor']['orientation'] = dict(x=0., y=0., z=0., w=1.)
    order = sorted(('left', 'right'), key=lambda payload: stopped['poses']['box48_' + payload]['position'].get('y', 0))
    stopped.update(mode='retrieval_pickup', assignment=dict(right=order[0], left=order[1]), retrieval=True)
    stopped['nominal_phase_fixture_is_prospective_not_actual_new_stop'] = True
    from dense_fixture_scope import declare_prospective_empty_holder_scope
    stopped = declare_prospective_empty_holder_scope(stopped)"""
    source = source.replace(marker, replacement)
    for before, after in {'paired_fixtures': 'paired_fixtures_nominal_phase',
                          'right_retrieval_pickup_request.json': 'left_right_pickup_request.json',
                          'paired_fixture_audit.json': 'paired_fixture_nominal_phase_audit.json'}.items():
        assert source.count(before) == 1
        source = source.replace(before, after)
    with (EVIDENCE / 'nominal_stop_fixture_provenance.json').open('x') as stream:
        json.dump(dict(status='PROSPECTIVE_ENCODER_PHASE_ZERO_FIXTURE_NOT_MEASURED_STOP',
                       source_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),
                       executed_sha256=hashlib.sha256(source.encode()).hexdigest(),
                       source_request_and_all_real_scene_poses_unchanged=True,
                       no_reset_pose_set_or_controller_command=True,
                       actual_stop_phase_and_actual_grasp_paths_still_required=True), stream, indent=2)
    exec(compile(source, str(original), 'exec'), dict(__name__='__main__', __file__=str(original)))


if __name__ == '__main__':
    main()
