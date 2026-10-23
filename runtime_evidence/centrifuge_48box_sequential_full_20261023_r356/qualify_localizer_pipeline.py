import hashlib
import json
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from unittest.mock import patch

from inputs.localizer_runtime import InstrumentedLocalizer, ORIGINAL_SHA256, original
from localization_freshness import localization_freshness
from localization_source import restore_localizer_launch


ROOT = Path(__file__).resolve().parent


class Records:
    def __init__(self):
        self.rows = []

    def record(self, event, **payload):
        self.rows.append(dict(event=event, **payload))


def main():
    parent = ROOT.parent / 'centrifuge_48box_sequential_full_20261021_r336'
    expected = (parent / 'inputs/lab_dual.launch.py').read_text().replace(str(parent), str(ROOT))
    assert restore_localizer_launch((ROOT / 'inputs/lab_dual.launch.py').read_text()) == expected
    assert hashlib.sha256(Path(original.__file__).read_bytes()).hexdigest() == ORIGINAL_SHA256
    assert (ROOT / 'localization_freshness.py').read_bytes() == (parent / 'localization_freshness.py').read_bytes()
    assert not localization_freshness(172.6025, 171.6, 2., 1., 1., True)['valid']
    recorder = Records()
    rclpy.init(args=['--ros-args', '-p', 'localization_rate:=5.0'])
    node = InstrumentedLocalizer(recorder)
    published = []
    try:
        transform = TransformStamped()
        transform.header.frame_id = 'pelvis'
        transform.child_frame_id = 'test_lidar'
        transform.transform.rotation.w = 1.
        node.transform_buffer.set_transform_static(transform, 'qualification_only')
        coordinates = np.linspace(-2.45, 2.45, 240)
        points = [(2.5, value, .2) for value in coordinates]
        points += [(-2.5, value, .2) for value in coordinates]
        points += [(value, 2.5, .2) for value in coordinates]
        points += [(value, -2.5, .2) for value in coordinates]
        with patch.object(node.pose_publisher, 'publish', side_effect=published.append):
            for index in range(51):
                header = Header(frame_id='test_lidar')
                header.stamp.sec = 100 + index // 10
                header.stamp.nanosec = (index % 10) * 100000000
                node._point_cloud_callback(create_cloud_xyz32(header, points))
        rows = [row for row in recorder.rows if row['event'] == 'pose_published']
        stamps = [row['stamp'] for row in rows]
        assert len(stamps) >= 17 and len(stamps) == len(published)
        assert max(np.diff(stamps)) <= .30000001
        assert all(abs(message.pose.pose.position.x) < .001 and
                   abs(message.pose.pose.position.y) < .001 for message in published)
        assert [message.header.stamp.sec + message.header.stamp.nanosec * 1.e-9
                for message in published] == stamps
        assert all(row['valid'] for row in recorder.rows if row['event'] == 'status_published')
        assert sum(row['event'] == 'tf_exit' for row in recorder.rows) == 51
        assert sum(row['event'] == 'cloud_enter' for row in recorder.rows) == 51
        assert node.localization_rate == 5. and node.transform_timeout == 1.
        result = dict(status='PASS_ORIGINAL_ALGORITHM_SYNTHETIC_CLOUDS_NOT_PHYSICAL',
                      original_sha256=ORIGINAL_SHA256, original_guard_unchanged=True,
                      clouds=51, published_poses=len(stamps), max_pose_gap_sim_s=max(np.diff(stamps)),
                      original_source_stamps_preserved=True, actual_records=recorder.rows,
                      sole_launch_delta='diagnostic_subclass_and_5Hz_rate',
                      private_rate_gate='integer_nanoseconds_only', formal_source_unchanged=True)
        with (ROOT / 'localizer_pipeline_qualification.json').open('x') as stream:
            json.dump(result, stream, indent=2)
        print(result['status'], len(stamps), max(np.diff(stamps)), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
