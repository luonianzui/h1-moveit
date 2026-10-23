import gzip
import hashlib
import json
from pathlib import Path


EVIDENCE = Path(__file__).resolve().parent


def main():
    intervals = []
    milestones = []
    velocity_peak = None
    count = 0
    selected = {
        'LOADING_PASS', 'MACHINE_CYCLE_PASS', 'PASS', 'FAIL', 'steady_start',
        'phase_aligned_stop_schedule', 'actual_stop_handover_dense_pass',
        'return_table_target', 'dual_holder_normal_release', 'table_return_pass',
        'leg_posture_declared', 'leg_posture_pass', 'command',
    }
    with gzip.open(EVIDENCE / 'run/loading_trace.jsonl.gz', 'rb') as stream:
        for raw in stream:
            header, separator, remainder = raw.partition(b', "payload": ')
            row = json.loads(header + b'}') if separator else json.loads(raw)
            count += 1
            if row['sim'] is not None:
                if not intervals or row['phase'] != intervals[-1]['phase']:
                    if intervals:
                        intervals[-1]['end_sim'] = row['sim']
                        intervals[-1]['end_wall'] = row['wall']
                    intervals.append(dict(phase=row['phase'], start_sim=row['sim'],
                                          start_wall=row['wall']))
                intervals[-1]['last_sim'] = row['sim']
                intervals[-1]['last_wall'] = row['wall']
            if row['kind'] in selected:
                full = json.loads(raw)
                if row['kind'] == 'command':
                    payload = full['payload']
                    full['payload'] = {key: value for key, value in payload.items()
                                       if not isinstance(value, (dict, list))}
                    full['payload_keys'] = list(payload)
                milestones.append(full)
            if row['kind'] == 'raw_gripper_velocity':
                payload = json.loads(raw)['payload']
                for joint, velocity in payload['velocities'].items():
                    if velocity_peak is None or abs(velocity) > abs(velocity_peak['velocity_m_s']):
                        velocity_peak = dict(joint=joint, velocity_m_s=velocity,
                                             source_sim=payload['joint_sim'], **row)
    for interval in intervals:
        interval.setdefault('end_sim', interval['last_sim'])
        interval.setdefault('end_wall', interval['last_wall'])
        interval['duration_sim_s'] = interval['end_sim'] - interval['start_sim']
    result = dict(status='OBSERVED_COMPLETE_EXECUTION_NOT_180_AUDIT' if any(row['kind'] == 'PASS' for row in milestones)
                  else 'OBSERVED_PREFIX_NOT_COMPLETE_PASS', rows=count,
                  trace_sha256=hashlib.sha256((EVIDENCE / 'run/trace_metadata.json').read_bytes()).hexdigest(),
                  phase_intervals=intervals, milestones=milestones,
                  maximum_actual_gripper_velocity=velocity_peak,
                  first_nonempty_sim=intervals[0]['start_sim'],
                  exact_video_sim_synchronization_not_claimed=True)
    with (EVIDENCE / 'run/actual_timing_profile.json').open('x') as output:
        json.dump(result, output, indent=2)
    print('ACTUAL_PHASE_PROFILE', count, len(intervals), flush=True)
    for interval in intervals:
        print(interval['phase'], round(interval['duration_sim_s'], 6), flush=True)


if __name__ == '__main__':
    main()
