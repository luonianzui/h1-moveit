import math

from h1_2_navigation.mid360_square_localizer import (
    Mid360SquareLocalizer, Time, Duration, TransformException, np,
    _point_cloud_xyz, transform_points, filter_navigation_points,
    select_square_wall_candidates, refine_square_pose, evaluate_square_pose,
    square_quality_passes,
)


def localization_due(stamp_ns, previous_ns, rate):
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError('Localization rate must be finite and positive')
    return previous_ns is None or stamp_ns < previous_ns or stamp_ns - previous_ns >= math.ceil(1.e9 / rate)


class ExactRateLocalizer(Mid360SquareLocalizer):
    def _point_cloud_callback(self, message):
        try:
            transform = self.transform_buffer.lookup_transform(
                self.base_frame,
                message.header.frame_id,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=self.transform_timeout),
            )
            points = _point_cloud_xyz(message)
        except (TransformException, ValueError) as error:
            self._publish_status(False, f'input_error:{error}', 0)
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        points = transform_points(
            points,
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )
        scan_points = filter_navigation_points(
            points,
            self.minimum_height,
            self.maximum_height,
            self.scan_minimum_range,
            self.maximum_range,
        )
        self._publish_scan(message, scan_points)
        localization_points = filter_navigation_points(
            scan_points,
            self.minimum_height,
            self.maximum_height,
            self.minimum_range,
            self.maximum_range,
        )[:, :2]
        localization_points = select_square_wall_candidates(
            localization_points,
            self.pose,
            self.wall_half_extent,
            self.localization_candidate_band,
        )
        if len(localization_points) < self.minimum_point_count:
            self._publish_status(
                False, 'insufficient_wall_candidates',
                len(localization_points))
            return
        stamp_seconds = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1.0e-9)
        stamp_ns = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        last_ns = getattr(self, 'last_localization_stamp_ns', None) if self.last_localization_stamp is not None else None
        if not localization_due(stamp_ns, last_ns, self.localization_rate):
            return
        self.last_localization_stamp_ns = stamp_ns
        self.last_localization_stamp = stamp_seconds
        if len(localization_points) > self.maximum_localization_points:
            indices = np.linspace(
                0,
                len(localization_points) - 1,
                self.maximum_localization_points,
                dtype=np.int64,
            )
            localization_points = localization_points[indices]
        estimated_pose, robust_rms = refine_square_pose(
            localization_points,
            self.pose,
            self.wall_half_extent,
        )
        inlier_fraction, inlier_rms, side_counts = evaluate_square_pose(
            localization_points,
            estimated_pose,
            self.wall_half_extent,
            self.inlier_threshold,
        )
        valid, visible_wall_count = square_quality_passes(
            inlier_fraction,
            inlier_rms,
            side_counts,
            self.minimum_inlier_fraction,
            self.maximum_inlier_rms,
            self.minimum_points_per_wall,
            self.minimum_visible_walls,
        )
        reason = 'pass' if valid else 'quality_gate_failed'
        map_to_odom_published = False
        if valid:
            self.pose = estimated_pose
            try:
                map_to_odom_published = self._publish_pose(
                    message, inlier_rms)
            except TransformException as error:
                valid = False
                reason = f'odom_transform_unavailable:{error}'
        self._publish_status(
            valid,
            reason,
            len(localization_points),
            robust_rms,
            inlier_fraction,
            inlier_rms,
            side_counts,
            estimated_pose,
            visible_wall_count,
            map_to_odom_published,
        )

