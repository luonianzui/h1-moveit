def restore_initial_runtime_source(name, source):
    if name != 'execute_cycle.py':
        return source
    current = 'from initial_approach_cycle import InitialApproachCycle as DualCycle'
    original = 'from gentle_setdown_cycle import GentleSetdownCycle as DualCycle'
    if source.count(current) != 1:
        raise ValueError('Only the latest initial approach alias may be reversed')
    return source.replace(current, original, 1)
