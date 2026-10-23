import json
from pathlib import Path

import psutil


ROOT = Path(__file__).resolve().parent
identity = json.loads((ROOT / 'run/launch_pid.json').read_text())
owner = psutil.Process(identity['pid'])
assert owner.create_time() == identity['created']
records = []
for process in [owner] + owner.children(recursive=True):
    command = process.cmdline()
    if not any('parameter_bridge' in part or 'robot_joint_state_relay_05ms.py' in part for part in command):
        continue
    environment = process.environ()
    for key, expected in dict(RMW_IMPLEMENTATION='rmw_fastrtps_cpp', FASTDDS_BUILTIN_TRANSPORTS='UDPv4',
                              ROS_DOMAIN_ID='165', GZ_PARTITION='h1_seq_full_r357').items():
        assert environment.get(key) == expected, (process.pid, key)
    records.append(dict(pid=process.pid, created=process.create_time(), command=command,
                        transport=environment['FASTDDS_BUILTIN_TRANSPORTS']))
assert any('robot_joint_state_relay_05ms.py' in ' '.join(row['command']) for row in records)
assert any('parameter_bridge' in ' '.join(row['command']) for row in records)
with (ROOT / 'run/transport_environment.json').open('x') as stream:
    json.dump(dict(status='PASS_OWNED_PROCESS_ENV_NOT_PACKET_LOSS_PROOF', processes=records), stream, indent=2)
print('TRANSPORT_PROCESS_ENV_PASS', len(records), flush=True)
