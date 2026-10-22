import json
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


TOPICS = ('/mid360_localization/pose', '/mid360_localization/status')


def latest_localization_qos():
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


def delivery_metadata(kind, message, information, clock_sim):
    source_ns = int(information.get('source_timestamp', 0) if isinstance(information, dict)
                    else getattr(information, 'source_timestamp', 0))
    received_ns = int(information.get('received_timestamp', 0) if isinstance(information, dict)
                      else getattr(information, 'received_timestamp', 0))
    callback_ns = time.time_ns()
    stamp = message.header.stamp if kind == 'pose' else None
    return dict(kind=kind, source_sim=None if stamp is None else stamp.sec + stamp.nanosec * 1e-9,
                clock_sim=clock_sim, source_timestamp_ns=source_ns,
                received_timestamp_ns=received_ns, callback_wall_ns=callback_ns,
                callback_monotonic_ns=time.monotonic_ns(),
                transport_wall_s=(received_ns-source_ns)/1e9 if source_ns and received_ns else None,
                callback_queue_wall_s=(callback_ns-received_ns)/1e9 if received_ns else None,
                dds_timestamps_available=bool(source_ns and received_ns),
                source_stamp_not_rewritten=True)


def configure_localization_delivery(cycle):
    selected = [subscription for subscription in cycle.node.subscriptions
                if subscription.topic_name in TOPICS]
    if sorted(subscription.topic_name for subscription in selected) != sorted(TOPICS):
        raise RuntimeError('Expected exactly the original two localization subscriptions')
    for subscription in selected:
        if not cycle.node.destroy_subscription(subscription):
            raise RuntimeError('Could not remove original localization subscription')

    def receive_pose(message, information):
        cycle.record('localization_delivery', delivery_metadata('pose', message, information, cycle.sim))
        cycle.receive_localization_pose(message)

    def receive_status(message, information):
        cycle.record('localization_delivery', delivery_metadata('status', message, information, cycle.sim))
        cycle.receive('localization', json.loads(message.data))

    cycle.localization_delivery_subscriptions = [
        cycle.node.create_subscription(PoseWithCovarianceStamped, TOPICS[0], receive_pose, latest_localization_qos()),
        cycle.node.create_subscription(String, TOPICS[1], receive_status, latest_localization_qos()),
    ]
    cycle.record('localization_delivery_contract', dict(
        topics=TOPICS, history='KEEP_LAST', depth=1, reliability='BEST_EFFORT', durability='VOLATILE',
        reliable_publisher_compatible=True, no_reliable_head_of_line_repair_wait=True,
        sensor_quality_algorithm_and_publish_rate_unchanged=True,
        original_source_freshness_limit_sim_s=1., no_missing_pose_substitution=True,
        no_extra_motion_or_grip_commands=True))
