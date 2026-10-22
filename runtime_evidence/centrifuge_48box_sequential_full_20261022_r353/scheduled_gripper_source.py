def restore_schedule_source(name, source):
    changes = {
        'execute_fast.py': [
            ('    def future_start_offset(self):\n        return .3\n\n', '', 1),
            ('start = self.sim + self.future_start_offset() if scheduled is None else scheduled',
             'start = self.sim + .3 if scheduled is None else scheduled', 1)],
        'execute_dual.py': [
            ('start = self.sim + self.future_start_offset()', 'start = self.sim + .3', 2),
            ('future_start_and_completion_margin_sim_s=self.future_start_offset() + .15',
             'future_start_and_completion_margin_sim_s=.45', 1),
            ('command_paths_rates_and_sim_timestamps_unchanged=self.future_start_offset() == .3',
             'command_paths_rates_and_sim_timestamps_unchanged=True', 1)],
        'efficient_gripper_cycle.py': [
            ('    def qualified_short_stroke_last_time(self, original_last_time):\n        return original_last_time\n\n', '', 1),
            ('        last_time = self.qualified_short_stroke_last_time(last_time)\n', '', 1),
            ('        points[-1].time_from_start.sec, points[-1].time_from_start.nanosec = divmod(round(last_time * 1e9), 1000000000)\n',
             '        points[-1].time_from_start.sec = 0\n        points[-1].time_from_start.nanosec = 500000000\n', 1)],
        'execute_cycle.py': [
            ('from scheduled_gripper_cycle import ScheduledGripperCycle as DualCycle',
             'from ramped_button_cycle import RampedButtonCycle as DualCycle', 1)],
    }
    for current, original, count in changes.get(name, []):
        if source.count(current) != count:
            raise ValueError('Declared scheduling source hook missing or duplicated: ' + name)
        source = source.replace(current, original)
    return source
