import hashlib
import json
from pathlib import Path
import sys


EVIDENCE = Path(__file__).resolve().parent
REFERENCE = EVIDENCE.parent / 'centrifuge_48box_coupled_path_20261013_r240'


def digest(path):
    hasher = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            hasher.update(chunk)
    return hasher.hexdigest()


def main():
    phase = sys.argv[1]
    assert phase in ('before', 'after')
    source = REFERENCE / 'verify_protection.py'
    manifest = dict((relative.lstrip('*'), checksum) for checksum, relative in
                    (line.split(maxsplit=1) for line in (REFERENCE / 'SHA256SUMS').read_text().splitlines()))
    assert digest(source) == manifest[source.name]
    checked_source = source.read_text()
    previous = sys.argv[:]
    try:
        sys.argv = [str(EVIDENCE / 'verify_protection.py'), phase]
        exec(compile(checked_source, str(source), 'exec'),
             dict(__name__='__main__', __file__=str(EVIDENCE / 'verify_protection.py')))
    finally:
        sys.argv = previous
    failures, counts, hashes = [], {}, {}
    for name in ('centrifuge_48box_alignment_cap_20261013_r236',
                 'centrifuge_48box_parallel_transport_20261013_r238',
                 'centrifuge_48box_feedback_history_20261013_r239',
                 'centrifuge_48box_coupled_path_20261013_r240',
                 'centrifuge_48box_dual_runtime_20261013_r241',
                 'centrifuge_48box_dual_review_20261013_r242',
                 'centrifuge_48box_empty_contact_fix_20261013_r243',
                 'centrifuge_48box_camera_retry_20261013_r244',
                 'centrifuge_48box_clock_budget_20261013_r245',
                 'centrifuge_48box_empty_retreat_20261013_r246',
                 'centrifuge_48box_startup_window_20261013_r247',
                 'centrifuge_48box_paired_setdown_20261013_r248',
                 'centrifuge_48box_retrieval_approach_20261013_r249',
                 'centrifuge_48box_table_clearance_20261013_r250',
                 'centrifuge_48box_retrieval_medium_20261013_r251',
                 'centrifuge_48box_gain_calibration_20261013_r252',
                 'centrifuge_48box_gain_reverse_calibration_20261013_r253',
                 'centrifuge_48box_gain_full_cycle_20261013_r254',
                 'centrifuge_48box_empty_return_gain_20261014_r255',
                 'centrifuge_48box_stop_handover_20261014_r256',
                 'centrifuge_48box_loaded_dispatch_20261014_r257',
                 'centrifuge_48box_gripper_damping_20261014_r258',
                 'centrifuge_48box_camera_deadline_20261014_r259',
                 'centrifuge_48box_zero_effort_timing_20261014_r260',
                 'centrifuge_48box_leg_fresh_endpoint_20261014_r261',
                 'centrifuge_48box_complete_rate_profile_20261014_r262',
                 'centrifuge_48box_button_ramp_20261014_r263',
                 'centrifuge_48box_paired_ramp_20261014_r264',
                 'centrifuge_48box_retrieval_rate_restore_20261014_r265',
                 'centrifuge_48box_retrieval_rate_restore_20261014_r266',
                 'centrifuge_48box_direct_retrieval_20261014_r267',
                 'centrifuge_48box_schedule_gripper_20261014_r268',
                 'centrifuge_48box_native_gripper_feedback_20261014_r269',
                 'centrifuge_48box_dense_shortcut_20261014_r270',
                 'centrifuge_48box_dense_shortcut_full_20261014_r271',
                 'centrifuge_48box_r271_protection_20261014_r1',
                 'centrifuge_48box_gentle_setdown_20261014_r272',
                 'centrifuge_48box_initial_approach_20261014_r273',
                 'centrifuge_48box_startup_ramp_probe_20261014_r1',
                 'centrifuge_48box_startup_ramp_full_20261014_r274',
                 'centrifuge_48box_retrieval_grip_full_20261014_r275',
                 'centrifuge_48box_pickup_approach_full_20261014_r276',
                 'centrifuge_48box_empty_gripper_return_20261014_r277',
                 'centrifuge_48box_post_retreat_gripper_20261014_r278',
                 'centrifuge_48box_grip_height_probe_20261014_r1',
                 'centrifuge_48box_coarse_ramp_probe_20261014_r1',
                 'centrifuge_48box_coarse_ramp_full_20261014_r279',
                 'centrifuge_48box_empty_protocol_probe_20261015_r1',
                 'centrifuge_48box_empty_protocol_full_20261015_r280',
                 'centrifuge_48box_dense_tree_probe_20261015_r1',
                 'centrifuge_48box_endpoint_overlap_probe_20261015_r1',
                 'centrifuge_48box_dense_endpoint_full_20261015_r281',
                 'centrifuge_48box_lift_density_probe_20261015_r1',
                 'centrifuge_48box_bucket_axis_exit_probe_20261015_r1',
                 'centrifuge_48box_bucket_axis_exit_full_20261015_r282',
                 'centrifuge_48box_optional_alignment_retreat_probe_20261015_r1',
                 'centrifuge_48box_micro_alignment_probe_20261015_r1',
                 'centrifuge_48box_optional_micro_full_20261015_r283',
                 'centrifuge_48box_front_camera_full_20261015_r284',
                 'centrifuge_48box_front_briefing_20261015_r1',
                 'centrifuge_48box_minimum_turn_full_20261015_r285',
                 'centrifuge_48box_minimum_turn_briefing_20261015_r1',
                 'centrifuge_48box_elbow_posture_probe_20261016_r1',
                 'centrifuge_48box_elbow_full_20261016_r286',
                 'centrifuge_48box_elbow_full_20261016_r287',
                 'centrifuge_48box_elbow_full_20261016_r288',
                 'centrifuge_48box_elbow_full_20261016_r289',
                 'centrifuge_48box_elbow_full_20261016_r290',
                 'centrifuge_48box_wallclock_perf_20261015_r1',
                 'centrifuge_48box_wallclock_perf_20261015_r2',
                 'centrifuge_48box_wallclock_perf_20261015_r3',
                 'centrifuge_48box_wallclock_profile_20261015_r4',
                 'centrifuge_48box_review_video_10x_20261015_r1',
                 'centrifuge_48box_briefing_cut_20261015_r2',
                 'build_cache_cleanup_20261014_r1',
                 'trace_lossless_codec_probe_20261014_r1',
                 'video_cleanup_20261014_r1',
                 'video_cleanup_20261014_r2',
                 'installer_download_cleanup_20261014_r1',
                 'trash_cache_purge_20261014_r1'):
        root = EVIDENCE.parent / name
        rows = (root / 'SHA256SUMS').read_text().splitlines()
        counts[name] = len(rows)
        hashes[name] = digest(root / 'SHA256SUMS')
        for line in rows:
            expected, relative = line.split(maxsplit=1)
            path = root / relative.lstrip('*')
            if digest(path) != expected or path.stat().st_mode & 0o777 != 0o444:
                failures.append(str(path))
        if name == 'centrifuge_48box_gripper_damping_20261014_r258':
            erratum = root / 'REPORT_ERRATUM.sha256'
            erratum_digest = digest(erratum)
            if erratum_digest != '69f8d7f703dad1767dec1e6102c064f916e75a84db2d0e2d6c6424d23ef9b86b':
                failures.append(str(erratum))
            expected, relative = erratum.read_text().strip().split(maxsplit=1)
            path = root / relative.lstrip('*')
            if digest(path) != expected or path.stat().st_mode & 0o777 != 0o444 or erratum.stat().st_mode & 0o777 != 0o444:
                failures.append(str(path))
            hashes[name + '/REPORT_ERRATUM.sha256'] = erratum_digest
            counts[name] += 2
    result = dict(status='PASS' if not failures else 'FAIL', counts=counts, manifests=hashes, failures=failures)
    if phase == 'after':
        assert result == json.loads((EVIDENCE / 'dual_protected_before.json').read_text())
    with (EVIDENCE / ('dual_protected_' + phase + '.json')).open('x') as stream:
        json.dump(result, stream, indent=2)
    assert not failures
    print('DUAL_PROTECTION_PASS', phase, counts, flush=True)


if __name__ == '__main__':
    main()
