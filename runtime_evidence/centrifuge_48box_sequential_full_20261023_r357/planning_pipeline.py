import copy
from pathlib import Path

from joined_stages import joined_phase, joined_samples, stage_groups
from planner_client import PlannerClient


class PlanningPipelineMixin:
    def run_complete(self):
        self.planning_worker = PlannerClient(Path(__file__).resolve().parent)
        return super().run_complete()

    def close(self):
        try:
            if getattr(self, 'planning_worker', None) is not None:
                self.planning_worker.close()
        finally:
            super().close()

    def execute_stages(self, planned, prefix):
        for group in stage_groups(planned['stages']):
            if len(group) == 1:
                single = dict(planned, stages=group)
                super().execute_stages(single, prefix)
                continue
            if len(self.holders.rows) != 1:
                raise RuntimeError('Joined loaded segments require exactly one actual holder')
            self.joined_stage_segments = copy.deepcopy(group)
            try:
                self.record('joined_segments_declared', dict(prefix=prefix,
                    segments=[stage['phase'] for stage in group], original_dense_samples_preserved=True,
                    per_segment_acceleration_ramps_preserved=True, no_contact_or_observation_boundary_removed=True))
                self.move(joined_samples(group), prefix + '_' + joined_phase(group), .25)
                self.phase = prefix + '_' + group[-1]['phase']
                self.record('joined_segments_completed', dict(final_original_phase=self.phase,
                    original_final_endpoint_feedback_passed=True))
            finally:
                self.joined_stage_segments = None
