OLD_ENTRY = """    from sequential_cycle import SequentialCycle
    cycle = SequentialCycle(plan, buttons)"""
NEW_ENTRY = """    from speed_policy import PROFILE, configure_simulation_profile
    assert identity['speed_profile'] == PROFILE
    configure_simulation_profile()
    from table_alignment_cycle import SpeedPriorityCycle
    cycle = SpeedPriorityCycle(plan, buttons)"""


def restore_speed_source(name, source):
    if name == 'trace_io.py':
        source = source.replace('compresslevel=1', 'compresslevel=6')
        source = source.replace("compression='gzip_level_1'", "compression='gzip_level_6'")
    if name == 'execute_loading.py':
        from bucket_cache_source import CACHED_BUCKET_FUNCTION, ORIGINAL_BUCKET_FUNCTION
        source = source.replace(CACHED_BUCKET_FUNCTION, ORIGINAL_BUCKET_FUNCTION)
        from pose_cache_source import CACHED_POSE_FUNCTION, ORIGINAL_POSE_FUNCTION
        source = source.replace(CACHED_POSE_FUNCTION, ORIGINAL_POSE_FUNCTION)
        source = source.replace('from batched_trace import BatchedLosslessTrace as LosslessTrace',
                                'from trace_io import LosslessTrace')
        source = source.replace("getattr(self, 'pose_clock_skew_limit_sim_s', .15)", '.15')
        source = source.replace("getattr(self, 'gripper_velocity_violation', velocity_violation)(value)", 'velocity_violation(value)')
    if name == 'execute_sequential.py':
        source = source.replace("identity['actual_runtime_subclass'] == 'SpeedPriorityCycle'",
                                "identity['actual_runtime_subclass'] == 'SequentialCycle'")
        source = source.replace(NEW_ENTRY, OLD_ENTRY)
    return source
