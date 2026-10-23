from minimum_turn_source import restore_minimum_turn
from elbow_source import restore_elbow
from elbow_empty_source import restore_elbow_empty
from elbow_transport_source import restore_elbow_transport


def restore_inherited_source(name, source):
    from acceleration_source import restore_acceleration
    source = restore_acceleration(name, source)
    source = restore_elbow_transport(name, source)
    source = restore_elbow_empty(name, source)
    source = restore_elbow(name, source)
    source = restore_minimum_turn(name, source)
    if not name.endswith('.py') or '/' in name or '\\' in name:
        raise ValueError('Only inherited top-level source comparisons are declared')
    if name == 'dual_worker.py':
        from micro_alignment_source import restore_micro_alignment
        return restore_micro_alignment(source)
    if name in ('post_retreat_gripper_cycle.py','gripper_return_geometry.py'):
        source = source.replace('from post_retreat_contract import RETREAT_PHASES, expected_retreat_phases',
                                'from post_retreat_contract import RETREAT_PHASES',1)
        if name == 'post_retreat_gripper_cycle.py':
            source = source.replace("        expected = expected_retreat_phases(getattr(self, 'qualified_empty_retreat_guides', None))\n        if tuple(stage['phase'] for stage in stages) != expected + ('empty_return',):",
                "        if tuple(stage['phase'] for stage in stages) != RETREAT_PHASES + ('empty_return',):",1)
            return source.replace(".13 if stage['phase'] == RETREAT_PHASES[0] else .8",'.13 if index == 0 else .8',1)
        return source.replace("expected_retreat_phases(request.get('current_run_executed_loading_guides'))",'RETREAT_PHASES',1)
    return source
