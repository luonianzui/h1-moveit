import json
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from std_msgs.msg import String

from localization_delivery import TOPICS, configure_localization_delivery, delivery_metadata, latest_localization_qos
from localization_freshness import localization_freshness


class FakeNode:
    def __init__(self, topics=TOPICS):
        self.subscriptions = [SimpleNamespace(topic_name=topic) for topic in (*topics, '/joint_states')]
        self.created = []

    def destroy_subscription(self, subscription):
        self.subscriptions.remove(subscription)
        return True

    def create_subscription(self, message_type, topic, callback, qos):
        subscription = SimpleNamespace(topic_name=topic, message_type=message_type, callback=callback, qos=qos)
        self.created.append(subscription)
        self.subscriptions.append(subscription)
        return subscription


def test_latest_qos_keeps_original_gate_and_reliable_publisher_compatibility():
    qos = latest_localization_qos()
    assert qos.depth == 1 and qos.history == HistoryPolicy.KEEP_LAST
    assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == DurabilityPolicy.VOLATILE
    assert latest_localization_qos() is not qos


def test_jazzy_dictionary_metadata_matches_attribute_form():
    information = dict(source_timestamp=100, received_timestamp=300)
    message = String(data='{}')
    output = delivery_metadata('status', message, information, 0.)
    assert output['dds_timestamps_available']
    assert output['transport_wall_s'] == 200 / 1e9


def test_replacement_is_limited_to_two_subscriptions_and_keeps_real_messages():
    events, poses, statuses = [], [], []
    cycle = SimpleNamespace(node=FakeNode(), sim=39.901, record=lambda *args: events.append(args),
                            receive_localization_pose=poses.append,
                            receive=lambda *args: statuses.append(args))
    configure_localization_delivery(cycle)
    assert len(cycle.node.created) == 2
    assert [item.topic_name for item in cycle.node.subscriptions].count('/joint_states') == 1
    pose = PoseWithCovarianceStamped()
    pose.header.stamp.sec, pose.header.stamp.nanosec = 38, 900000000
    metadata = SimpleNamespace(source_timestamp=100, received_timestamp=200)
    cycle.node.created[0].callback(pose, metadata)
    cycle.node.created[1].callback(String(data='{"valid":false,"reason":"quality_gate_failed"}'), metadata)
    assert poses == [pose] and poses[0] is pose
    assert statuses == [('localization', {'valid': False, 'reason': 'quality_gate_failed'})]
    assert events[1][1]['source_sim'] == 38.9
    assert not localization_freshness(39.901, 38.9, 3., 2., 2., True)['valid']
    assert not localization_freshness(39.901, 39.9, 3., 2., 2., False)['valid']


@pytest.mark.parametrize('topics', [(), TOPICS[:1], (*TOPICS, TOPICS[0])])
def test_missing_or_duplicate_original_subscription_is_rejected(topics):
    cycle = SimpleNamespace(node=FakeNode(topics))
    with pytest.raises(RuntimeError, match='exactly'):
        configure_localization_delivery(cycle)


@pytest.mark.parametrize('source,received', [(0, 0), (0, 42), (42, 0), (42, 52)])
def test_timing_metadata_does_not_invent_unavailable_delivery_times(source, received):
    output = delivery_metadata('status', String(data='{}'),
                               SimpleNamespace(source_timestamp=source, received_timestamp=received), 2.)
    assert output['dds_timestamps_available'] is bool(source and received)
    assert output['source_sim'] is None
    assert (output['transport_wall_s'] is None) is not bool(source and received)
    json.dumps(output, allow_nan=False)


@pytest.mark.parametrize('age', [1.000000001, 1.001, 2., 10., float('inf')])
def test_stale_localization_still_fails_without_wait_or_backfill(age):
    assert not localization_freshness(10.+age, 10., 3., 2., 2., True)['valid']
