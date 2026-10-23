from collections import Counter
from trace_io import open_trace
from cycle_guard import retrieval_ready_for_demo
from grip_limits import CARRY_ANGLE_LIMIT_DEG
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from execute_loading import fully_inside_bucket, pose_matrix
from loaded_retention import retention_metrics
from table_return_goal import table_orientation_error
from table_bucket_clearance import bucket_collision_bounds
from table_clearance_audit import verify_table_record

EVIDENCE = Path(__file__).resolve().parent
counts = Counter()
events = []
contacts = {'left': [], 'right': []}
latest_machine = None
latest_poses = None
loading_pass = None
machine_pass = None
final_pass = None
drop_pass = {}
retrieval_holds = {}
returns = {}
table_targets = {}
loaded = False
loaded_checks = 0
retention_checks = 0
maximum_side_overhang_m = 0.0
maximum_loaded_contact_depth_m = 0.0
maximum_bottom_penetration_m = 0.0
maximum_zero_force_gap_sim_s = 0.0
containment_faults = 0
localization_samples = 0
localization_invalid = 0
peak_rpm = 0.0
peak_slip = 0.0
peak_angle = 0.0
minimum_extraction_clearance_m = float('inf')
clearance_checks = 0
table_bucket_bounds = bucket_collision_bounds(EVIDENCE.parents[1] / 'src/h1_2_description/models/centrifuge/model.sdf')
table_clearance_checks = dict(left=0, right=0)
minimum_table_bucket_clearance_m = float('inf')
terminal_pass = None
terminal_samples = 0
terminal_failures = 0
failures = []
first_sim = None
relaxation = None
retrieval_samples = 0
retrieval_faults = 0
peak_retrieval_rpm = 0.0
original_quiet_exceeding_samples = 0
with open_trace(EVIDENCE) as stream:
    for line in stream:
        row = json.loads(line)
        kind, data = row['kind'], row['payload']
        counts[kind] += 1
        if first_sim is None and row['sim'] is not None:
            first_sim = row['sim']
        if kind == 'simulation_relaxation_declared':
            assert relaxation is None
            relaxation = data
        elif kind == 'contact':
            contacts[data['side']] = data['pairs']
        elif kind == 'physical_press':
            events.append(data)
        elif kind == 'drop_acceptance_pass':
            drop_pass[data['side']] = data
        elif kind == 'LOADING_PASS':
            loading_pass = row
            loaded = True
        elif kind == 'MACHINE_CYCLE_PASS':
            machine_pass = row
            loaded = False
        elif kind == 'retrieval_static_grip_hold_pass':
            retrieval_holds[data['payload']] = data
        elif kind == 'return_table_target':
            assert data['payload'] not in table_targets
            table_targets[data['payload']] = row
        elif kind == 'table_return_pass':
            returns[data['payload']] = row
        elif kind == 'machine':
            latest_machine = data
            peak_rpm = max(peak_rpm, abs(data['rotor_rpm']))
            if '_retrieval_' in row['phase']:
                retrieval_samples += 1
                retrieval_faults += not retrieval_ready_for_demo(data)
                peak_retrieval_rpm = max(peak_retrieval_rpm, abs(data['rotor_rpm']))
                original_quiet_exceeding_samples += abs(data['rotor_rpm']) > .02
            if data['rotor_enabled'] and abs(data['lid_rad']) >= math.radians(.2):
                failures.append('rotor enabled without closed lid')
        elif kind == 'localization':
            localization_samples += 1
            localization_invalid += not data['valid']
        elif kind == 'localization_health':
            if not data['valid'] or not -.15 <= data['pose_age_sim_s'] <= 1.0 or max(data['pose_age_wall_s'], data['status_age_wall_s']) > 60:
                failures.append('localization source freshness')
        elif kind == 'loaded_retention':
            retention_checks += 1
            for payload in ('left', 'right'):
                metrics = retention_metrics(data['poses'], data['contacts'][payload], payload)
                containment_faults += not (metrics['sidewall_certified'] and metrics['vertical_inside'] and metrics['contacts_valid'])
                maximum_side_overhang_m = max(maximum_side_overhang_m, metrics['side_overhang_m'])
                maximum_loaded_contact_depth_m = max(maximum_loaded_contact_depth_m, metrics['maximum_contact_depth_m'])
                maximum_bottom_penetration_m = max(maximum_bottom_penetration_m, metrics['bottom_penetration_m'])
        elif kind == 'support_continuity_sample':
            metrics = retention_metrics(data['poses'], data['contacts'], data['payload'])
            continuity = data['metrics']['support_continuity']
            containment_faults += not (metrics['contacts_valid'] and metrics['sidewall_certified'] and metrics['vertical_inside'] and continuity['valid'] and 0 <= continuity['zero_force_gap_sim_s'] <= .02)
            maximum_zero_force_gap_sim_s = max(maximum_zero_force_gap_sim_s, continuity['maximum_zero_force_gap_sim_s'])
        elif kind in ('synchronized_grip', 'retrieval_grip'):
            peak_slip = max(peak_slip, data['native_slip_m'], data['fk_slip_m'])
            peak_angle = max(peak_angle, data['native_angle_deg'], data['fk_angle_deg'])
        elif kind == 'state' and len(data['poses']) == 13:
            latest_poses = data['poses']
            if loaded:
                loaded_checks += 1
                for payload in ('left', 'right'):
                    containment_faults += any('dex1' in pair['first'] + pair['second'] for pair in contacts[payload])
        elif kind == 'extraction_clearance':
            clearance_checks += 1
            minimum_extraction_clearance_m = min(minimum_extraction_clearance_m, data['clearance']['clearance_m'])
            if not data['clearance']['valid']:
                failures.append('extraction_clearance_invalid')
        elif kind == 'table_setdown_bucket_clearance':
            verified = verify_table_record(row['phase'], data, table_bucket_bounds)
            table_clearance_checks[data['payload']] += 1
            minimum_table_bucket_clearance_m = min(minimum_table_bucket_clearance_m, verified['clearance_m'])
            if not verified['valid'] or data['contacts'] != contacts[data['payload']]:
                failures.append('independent_table_bucket_clearance_invalid')
        elif kind == 'terminal_table_hold_sample':
            terminal_samples += 1
            terminal_failures += data['passive_leg_deviation_rad'] > .025
            terminal_failures += sum(not value['continuity']['valid'] for value in data['rows'].values())
        elif kind == 'terminal_table_hold_pass':
            terminal_pass = data
        elif kind == 'PASS':
            final_pass = row
        if kind.endswith('_fault') or kind.endswith('_fault_sample') or kind == 'FAIL':
            failures.append(kind)
result = json.loads((EVIDENCE / 'run/result.json').read_text())
gates = dict(authorized_relaxation=relaxation is not None and relaxation.get('user_authorized') is True and relaxation.get('retrieval_transient_limit_rpm') == .1 and relaxation.get('original_strict_certificate_not_claimed') is True and relaxation.get('carry_angle_limit_deg') == CARRY_ANGLE_LIMIT_DEG,
             retrieval_transient_guard=retrieval_samples > 0 and retrieval_faults == 0,
             lossless_trace=json.loads((EVIDENCE / 'trace_audit.json').read_text())['status'] == 'PASS', setdown_braking=counts.get('retrieval_setdown_braking', 0) == 2, leg_posture=counts.get('leg_posture_pass', 0) == 1 and counts.get('leg_posture_goal_accepted', 0) == 1, executor=result['status'] == 'PASS', complete_final_event=final_pass is not None,
             both_loaded=loading_pass is not None and set(drop_pass) == {'left', 'right'},
             both_drop_stable=all(value['duration_sim_s'] >= 3 for value in drop_pass.values()),
             machine_cycle=machine_pass is not None,
             steady_30_seconds=machine_pass is not None and 30 <= machine_pass['payload']['steady_run_sim_seconds'] <= 32.4,
             physical_events=[event['button'] for event in events] == ['lid', 'lid', 'rotor', 'rotor', 'lid'] and
                 all(event['source'] == 'gazebo_physics_contact' for event in events),
             loaded_containment=loaded_checks > 0 and retention_checks > 0 and containment_faults == 0,
             both_retrieved=set(returns) == {'left', 'right'} and all(row['payload']['duration_sim_s'] >= 3 for row in returns.values()),
             retrieval_grip_holds=set(retrieval_holds) == {'left', 'right'} and
                 all(value['duration_sim_s'] >= 10 and value.get('hold_after_full_lift', False) for value in retrieval_holds.values()),
             full_extraction_clearance=clearance_checks > 0 and minimum_extraction_clearance_m >= .003,
             table_setdown_bucket_clearance=min(table_clearance_checks.values()) > 100 and minimum_table_bucket_clearance_m >= .003,
             terminal_hold_10_seconds=terminal_pass is not None and terminal_pass['duration_sim_s'] >= 10 and terminal_samples > 100 and terminal_failures == 0,
             final_stopped=latest_machine is not None and not latest_machine['rotor_enabled'] and abs(latest_machine['rotor_rpm']) <= .02,
             final_open=latest_machine is not None and abs(latest_machine['lid_rad'] - math.radians(120)) < math.radians(.2),
             one_start_stop=latest_machine is not None and latest_machine['starts'] == latest_machine['stops'] == 1,
             localization=localization_samples > 0 and localization_invalid == 0,
             grip_limits=peak_slip <= .002 and peak_angle < CARRY_ANGLE_LIMIT_DEG,
             no_failure=not failures, no_held_payload=result['holding'] is None and not result.get('actual_holder_registry', {'UNKNOWN':True}))
if latest_poses is not None:
    for payload in ('left', 'right'):
        hand = returns.get(payload, {}).get('payload', {}).get('hand')
        expected = np.array([.44, .38 if hand == 'left' else -.38, .84475])
        body = pose_matrix(latest_poses['box48_' + payload])
        gates[payload + '_final_table'] = (hand in ('left', 'right') and np.linalg.norm(body[:3, 3] - expected) < .002 and
            payload in table_targets and table_targets[payload]['sim'] < returns[payload]['sim'] and
            table_orientation_error(body, table_targets[payload]['payload']['yaw_rad']) < math.radians(2) and
            any('table::' in pair['first'] + pair['second'] for pair in contacts[payload]) and
            not any('dex1' in pair['first'] + pair['second'] for pair in contacts[payload]))
gates['distinct_return_slots'] = {row['payload']['hand'] for row in returns.values()} == {'left', 'right'}
audit = dict(status='PASS' if all(gates.values()) else 'FAIL', gates=gates,
             original_fixed_30_3_window_passed=machine_pass is not None and 30 <= machine_pass['payload']['steady_run_sim_seconds'] <= 30.3,
             user_minimum_30_steady_seconds_preserved=True,
             actual_empty_dual_registry=not result.get('actual_holder_registry', {'UNKNOWN':True}),
             scope='user_authorized_simulation_transient_guard_not_original_strict_certificate',
             authorized_relaxation=relaxation, retrieval_samples=retrieval_samples,
             retrieval_transient_faults=retrieval_faults, peak_retrieval_rpm=peak_retrieval_rpm,
             original_quiet_exceeding_samples=original_quiet_exceeding_samples,
             minimum_extraction_clearance_m=minimum_extraction_clearance_m, clearance_checks=clearance_checks,
             independently_recomputed_table_clearance_checks=table_clearance_checks,
             minimum_table_bucket_clearance_m=minimum_table_bucket_clearance_m,
             terminal_hold=terminal_pass, terminal_samples=terminal_samples, terminal_failures=terminal_failures,
             event_counts=dict(counts), loaded_pose_checks=loaded_checks, containment_faults=containment_faults,
             physical_press_events=events, localization_samples=localization_samples,
             localization_invalid=localization_invalid, peak_rpm=peak_rpm,
             peak_slip_m=peak_slip, peak_angle_deg=peak_angle, carry_angle_limit_deg=CARRY_ANGLE_LIMIT_DEG,
             original_carry_angle_gate_passed=peak_angle < 2,
             first_sim=first_sim, final_sim=final_pass['sim'] if final_pass else None,
             retention_checks=retention_checks, maximum_zero_force_gap_sim_s=maximum_zero_force_gap_sim_s, maximum_side_overhang_m=maximum_side_overhang_m,
             maximum_loaded_contact_depth_m=maximum_loaded_contact_depth_m, contact_geometry_rounding_m=.000005,
             original_contact_depth_limit_m=.001, contact_aware_geometry_bound_m=.001,
             maximum_bottom_penetration_m=maximum_bottom_penetration_m,
             certificate='same-basket contact manifold continuously present, positive support within 20ms, wall normal above 5um, original 1mm penetration bound',
             duration_sim_s=final_pass['sim'] - first_sim if final_pass else None,
             final_machine=latest_machine, failures=failures)
with (EVIDENCE / 'cycle_audit.json').open('x') as stream:
    json.dump(audit, stream, indent=2)
print(json.dumps(audit, indent=2))
assert audit['status'] == 'PASS'

