import json
import subprocess

from alignment_wait import wait_for_planner
from execute_dual import DualCycle, EVIDENCE
from handover_policy import FREE_PHASES, require_snapshot_drift, require_stop_pre, transport_rate


class StopHandoverCycle(DualCycle):
    def maximum_loaded_rate(self, phase):
        if phase in FREE_PHASES and set(self.holders.rows) == {'left', 'right'}:
            return .4
        return super().maximum_loaded_rate(phase)

    def free_transport_rate(self, phase, requested):
        return transport_rate(phase, requested, self.holders.rows)

    def move(self, samples, phase, rate):
        actual_rate = self.free_transport_rate(phase, rate)
        if actual_rate != rate:
            self.record('gain_free_transport_retiming_declared', dict(phase=phase, original_rate_rad_s=rate,
                        actual_rate_rad_s=actual_rate, two_actual_holders_required=True,
                        original_points_groups_floor_and_physical_guards_preserved=True,
                        only_motion_times_changed=True,
                        physical_speed_pass_not_preclaimed=True))
        return super().move(samples, phase, actual_rate)

    def check_actual_stop_handover(self):
        self.safety()
        if self.holders.rows or not self.values['machine']['can_open'] or self.values['machine']['rotor_enabled']:
            raise ValueError('Stopped empty-handed scene required before opening handover')
        pair = self.measured_pair()
        positions = dict(pair['joints']['positions'])
        require_stop_pre(positions, self.buttons['arms']['right'])
        request = dict(mode='stop_handover', positions=positions, source_sim=pair['pose_stamp'],
                       original_runtime_holder_references=self.holders.snapshot())
        self.phase = 'machine_stop_handover_collision_check'
        source, output = EVIDENCE / 'run/stop_handover_request.json', EVIDENCE / 'run/stop_handover_result.json'
        with source.open('x') as stream:
            json.dump(request, stream, indent=2)
        self.record('actual_stop_handover_audit_started', dict(source_sim=pair['pose_stamp'], request=str(source)))
        with (EVIDENCE / 'run/stop_handover_worker.log').open('x') as stream:
            process = subprocess.Popen(['/usr/bin/python3', str(EVIDENCE / 'stop_handover_worker.py'), str(source), str(output)],
                                       stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            wait_for_planner(process, self.spin, timeout_s=120)
        result = json.loads(output.read_text())
        assert result['status'] == 'PASS' and result['maximum_joint_step_rad'] == .002
        drift = require_snapshot_drift(self.positions, positions)
        self.record('actual_stop_handover_dense_pass', dict(audit=result, maximum_native_joint_numeric_drift=drift,
                    revolute_joint_drift_limit_rad=.0003, prismatic_joint_drift_limit_m=.0003))

    def press(self, side, phase, return_home=True, scheduled=None):
        if phase == 'machine_stop':
            if side != 'right' or scheduled is None or self.holders.rows:
                raise ValueError('Original scheduled physical right-hand stop required')
            event, start = super().press(side, phase, return_home=False, scheduled=scheduled)
            require_stop_pre(self.positions, self.buttons['arms']['right'])
            if not self.values['contact_status']['buttons']['rotor']['armed']:
                raise ValueError('Original rotor button rearm was lost before handover')
            self.record('stop_home_return_skipped', dict(original_physical_push_and_withdrawal_completed=True,
                        button_rearmed=self.values['contact_status']['buttons']['rotor']['armed'],
                        left_opening_requires_new_actual_dense_collision_audit=True))
            return event, start
        if phase == 'machine_final_open':
            self.check_actual_stop_handover()
        return super().press(side, phase, return_home=return_home, scheduled=scheduled)
