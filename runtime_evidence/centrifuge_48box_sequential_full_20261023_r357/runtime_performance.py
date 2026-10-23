from pathlib import Path
import json

from indexed_timeline import indexed_copy
from runtime_stack_sampler import RuntimeStackSampler


class RuntimePerformanceMixin:
    def __init__(self, plan, buttons):
        super().__init__(plan, buttons)
        self.joint_timeline = indexed_copy(self.joint_timeline)
        from localization_delivery import configure_localization_delivery
        configure_localization_delivery(self)

    def run_complete(self):
        sampler = RuntimeStackSampler(Path(__file__).resolve().parent / 'run/runtime_stack_profile.json',
                                      lambda: self.phase)
        sampler.start()
        try:
            return super().run_complete()
        finally:
            sampler.stop()
            from cached_pose import matrix_from_bytes
            with (Path(__file__).resolve().parent / 'run/pose_cache_stats.json').open('x') as stream:
                json.dump(dict(matrix_from_bytes.cache_info()._asdict(),
                               no_frame_decimation=True, no_pose_extrapolation=True), stream, indent=2)
            from cached_bucket import corners_from_keys
            with (Path(__file__).resolve().parent / 'run/bucket_cache_stats.json').open('x') as stream:
                json.dump(dict(corners_from_keys.cache_info()._asdict(),
                               no_contact_force_time_or_verdict_cached=True,
                               no_geometry_quantization=True), stream, indent=2)
