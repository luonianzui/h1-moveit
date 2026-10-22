#!/usr/bin/python3
import json
import math
import re


class PressLatch:
    def __init__(self):
        self.armed = False
        self.since = None
        self.last = None
        self.touching = False

    def update(self, stamp, touching):
        if not math.isfinite(stamp) or (self.last is not None and stamp < self.last):
            self.armed = False
            self.since = None
            self.last = None
            return False
        self.last = stamp
        if touching != self.touching or self.since is None:
            self.since = stamp
            self.touching = touching
        elapsed = stamp - self.since
        if not touching and elapsed >= .15:
            self.armed = True
        if touching and self.armed and elapsed >= .04:
            self.armed = False
            return True
        return False


def matching_contact(button, first, second, depths):
    cap = 'centrifuge::assembly::' + button + '_button_collision'
    finger = 'left_dex1_finger_1' if button == 'lid' else 'right_dex1_finger_2'
    pairs = [(first, second), (second, first)]
    valid_pair = any(own.endswith(cap) and other.startswith('h1_2::') and
                     re.search(r'(^|::)' + finger + r'(?:_|::)', other)
                     for own, other in pairs)
    return bool(valid_pair and depths and
                all(math.isfinite(depth) and 0 <= depth <= .002 for depth in depths))


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from ros_gz_interfaces.msg import Contacts
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('centrifuge_contact_buttons')
    action = node.create_publisher(String, '/centrifuge/button/action', 10)
    events = node.create_publisher(String, '/centrifuge/button/contact_event', 10)
    status = node.create_publisher(String, '/centrifuge/button/contact_status', 10)
    latches = {name: PressLatch() for name in ('lid', 'rotor')}
    contacts = {name: dict(stamp=-1.0, valid=False, pairs=[]) for name in latches}
    sequence = 0
    last_report = -1.0

    def receive(button, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        pairs = []
        for contact in message.contacts:
            first, second = contact.collision1.name, contact.collision2.name
            if matching_contact(button, first, second, contact.depths):
                pairs.append(dict(first=first, second=second, depths=list(contact.depths)))
        contacts[button] = dict(stamp=stamp, valid=bool(pairs), pairs=pairs)

    def clock(message):
        nonlocal sequence, last_report
        stamp = message.clock.sec + message.clock.nanosec * 1e-9
        for button, latch in latches.items():
            observation = contacts[button]
            touching = observation['valid'] and 0 <= stamp - observation['stamp'] <= .05
            if latch.update(stamp, touching):
                sequence += 1
                payload = dict(source='gazebo_physics_contact', button=button,
                               sequence=sequence, sim_time=stamp, contact=observation)
                events.publish(String(data=json.dumps(payload)))
                action.publish(String(data=button))
                node.get_logger().info(json.dumps(payload))
        if stamp - last_report >= .1:
            last_report = stamp
            status.publish(String(data=json.dumps(dict(sequence=sequence, sim_time=stamp,
                buttons={name: dict(armed=latch.armed, touching=latch.touching)
                         for name, latch in latches.items()}))))

    for button in latches:
        node.create_subscription(Contacts, '/centrifuge/' + button + '_button/contacts',
                                 lambda message, button=button: receive(button, message),
                                 qos_profile_sensor_data)
    node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
