def restore_gentle_runtime_source(name, source):
    if name != 'execute_cycle.py':
        return source
    current = 'from gentle_setdown_cycle import GentleSetdownCycle as DualCycle'
    original = 'from native_gripper_cycle import NativeGripperCycle as DualCycle'
    if source.count(current) != 1:
        raise ValueError('Only the declared gentle runtime alias may be reversed')
    return source.replace(current, original, 1)
