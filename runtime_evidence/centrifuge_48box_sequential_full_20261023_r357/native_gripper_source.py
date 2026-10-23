def restore_native_feedback_source(name, source):
    changes = {
        'execute_dual.py': [
            ('    def require_gripper_endpoint(self, hand, value, result_clock):\n        assert max(abs(self.positions[name] - value) for name in GROUP_JOINTS[hand + \'_gripper\']) < .00025\n\n', ''),
            ('        result_clock = self.sim\n        for hand, future in complete.items():\n',
             '        for hand, future in complete.items():\n'),
            ('            self.require_gripper_endpoint(hand, value, result_clock)\n',
             "            assert max(abs(self.positions[name] - value) for name in GROUP_JOINTS[hand + '_gripper']) < .00025\n")],
        'execute_cycle.py': [
            ('from native_gripper_cycle import NativeGripperCycle as DualCycle',
             'from scheduled_gripper_cycle import ScheduledGripperCycle as DualCycle')],
    }
    for current, original in changes.get(name, []):
        if source.count(current) != 1:
            raise ValueError('Only declared default-preserving native-feedback hooks may be reversed: ' + name)
        source = source.replace(current, original, 1)
    return source
