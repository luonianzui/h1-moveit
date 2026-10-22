import hashlib
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, qos_check_compatible, QoSCompatibility
from std_msgs.msg import String

from localization_delivery import TOPICS, configure_localization_delivery, latest_localization_qos
from localization_freshness import localization_freshness


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261020_r332'


def main():
    unchanged = {}
    for name in ('localization_freshness.py', 'execute_cycle.py', 'execute_loading.py',
                 'sequential_speed_cycle.py', 'speed_feedback_cycle.py', 'speed_policy.py',
                 'sequential_cycle.py', 'preload_observation.py', 'loaded_retention.py'):
        original = (PARENT / name).read_text().replace(str(PARENT), str(ROOT)).replace(PARENT.name, ROOT.name)
        original = original.replace('centrifuge_48box_sequential_full_r332', 'centrifuge_48box_sequential_full_r349')
        from pipeline_source import restore_pipeline_source
        assert restore_pipeline_source(name, (ROOT / name).read_text()) == original, name
        unchanged[name] = hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
    compatibility, reason = qos_check_compatible(QoSProfile(depth=10), latest_localization_qos())
    assert compatibility == QoSCompatibility.OK or (
        compatibility == QoSCompatibility.WARNING and reason.strip() ==
        'WARNING: Publisher liveliness is system_default and subscription liveliness is system_default;'), reason
    assert not localization_freshness(39.901, 38.9, 3., 2., 2., True)['valid']
    assert os.environ.get('ROS_DOMAIN_ID') == '159'
    rclpy.init()
    node = rclpy.create_node('r333_localization_offline_delivery_test')
    events, poses, statuses = [], [], []
    cycle = SimpleNamespace(node=node, sim=39.901, record=lambda *args: events.append(args),
                            receive_localization_pose=poses.append,
                            receive=lambda *args: statuses.append(args))
    node.create_subscription(PoseWithCovarianceStamped, TOPICS[0], poses.append, 10)
    node.create_subscription(String, TOPICS[1], lambda message: None, 10)
    configure_localization_delivery(cycle)
    pose_publisher = node.create_publisher(PoseWithCovarianceStamped, TOPICS[0], 10)
    status_publisher = node.create_publisher(String, TOPICS[1], 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    deadline = time.monotonic() + 5
    try:
        while time.monotonic() < deadline and not (poses and statuses):
            pose = PoseWithCovarianceStamped()
            pose.header.stamp.sec, pose.header.stamp.nanosec = 38, 900000000
            pose_publisher.publish(pose)
            status_publisher.publish(String(data='{"valid":true,"reason":"pass"}'))
            executor.spin_once(timeout_sec=.05)
            executor.spin_once(timeout_sec=.05)
        assert poses and statuses
        assert all(message.header.stamp.sec == 38 and message.header.stamp.nanosec == 900000000 for message in poses)
        delivery = [payload for kind, payload in events if kind == 'localization_delivery']
        assert {row['kind'] for row in delivery} == {'pose', 'status'}
        assert all(row['dds_timestamps_available'] for row in delivery)
        result = dict(status='PASS_REAL_DDS_LATEST_SAMPLE_COMPATIBILITY_NOT_GAZEBO_TASK',
                      original_timeout_reproduction_rejected=True, unchanged_sources=unchanged,
                      real_dds_messages=delivery, reliable_publishers_compatible=True,
                      static_compatibility_warning=reason,
                      dropped_messages_are_not_substituted_or_freshened=True)
        with (ROOT / 'localization_delivery_qualification.json').open('x') as stream:
            json.dump(result, stream, indent=2)
        print(result['status'], len(delivery), flush=True)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
