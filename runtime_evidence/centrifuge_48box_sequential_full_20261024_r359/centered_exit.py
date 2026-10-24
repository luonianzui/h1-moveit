import numpy as np

from axial_transport import SpeedRetrievalPlanner
from sequential_geometry import pose_matrix


class CenteredExitPlanner(SpeedRetrievalPlanner):
    def guided_stage(self, first, last, seed, phase, guides, table_end=False, support_start=None):
        if phase != 'retrieve_axis_exit':
            return super().guided_stage(first, last, seed, phase, guides, table_end, support_start)
        previous_checks = getattr(self, 'checked', 0)
        previous_tilt = getattr(self, 'maximum_payload_tilt_deg', 0.)
        try:
            return super().guided_stage(first, last, seed, phase, guides, table_end, support_start)
        except RuntimeError as error:
            if not str(error).startswith('Single-holder dense full-scene collision:'):
                raise
            initial_failure = str(error)
        rejected_checks = getattr(self, 'checked', 0) - previous_checks
        self.checked = previous_checks
        self.maximum_payload_tilt_deg = previous_tilt
        bucket = pose_matrix(self.request['poses']['centrifuge']) @ pose_matrix(self.request['poses']['bucket_' + self.payload])
        local = np.linalg.inv(bucket) @ first
        correction = -bucket[:3, 0] * local[0, 3]
        if not np.isfinite(correction).all() or np.linalg.norm(correction) > .002:
            raise RuntimeError('Original axis exit rejected; correction exceeds two millimeters')
        centered = np.array(first, copy=True)
        centered[:3, 3] += correction
        corrected_exit = np.array(last, copy=True)
        corrected_exit[:3, 3] += correction
        center_stage = self.stage(first, centered, seed, phase, count=11, refine=False)
        exit_stage = super().guided_stage(centered, corrected_exit, center_stage['samples'][-1],
                                         phase, guides, table_end=False, support_start=None)
        return dict(exit_stage, first_body=first.tolist(),
                    samples=center_stage['samples'] + exit_stage['samples'][1:],
                    clearance_correction=dict(original_axis_rejection=initial_failure,
                        rejected_candidate_dense_checks=rejected_checks,
                        translation_world_m=correction.tolist(), bucket_x_m=float(local[0, 3]),
                        maximum_correction_m=.002, full_scene_dense_checked=True,
                        no_basket_collision_exemption=True, orientation_unchanged=True,
                        original_holder_reference_unchanged=True))
