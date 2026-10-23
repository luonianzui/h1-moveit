import json
from pathlib import Path
import time

import rclpy
from std_msgs.msg import String

from startup_ramp_gate import verify_startup_status


ROOT = Path(__file__).resolve().parent


def main():
    rclpy.init()
    node = rclpy.create_node('startup_ramp_readonly_r274')
    received = []
    node.create_subscription(String, '/centrifuge/machine/status',
                             lambda message: received.append(dict(wall=time.time(), value=json.loads(message.data))), 10)
    deadline = time.monotonic() + 12
    validated = []
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            for sample in received:
                if sample['value'].get('ready') is True:
                    verify_startup_status(sample['value'])
                    validated.append(sample)
            received.clear()
            if len(validated) >= 2:
                with (ROOT / 'run/startup_ramp_readback.json').open('x') as stream:
                    json.dump(dict(status='PASS', actual_received_samples=validated,
                                   readonly_before_executor=True, no_machine_command_published=True), stream, indent=2)
                print('FRESH_STARTUP_RAMP_READBACK_PASS', len(validated), flush=True)
                return
        raise TimeoutError('Two fresh startup machine status samples were not received')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
