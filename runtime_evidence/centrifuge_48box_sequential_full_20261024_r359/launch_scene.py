import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess

import psutil
from scene_contract import read_contract

EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[1]
read_contract(EVIDENCE)
for name, expected in json.loads((EVIDENCE / 'identity.json').read_text())['sha256'].items():
    assert hashlib.sha256((EVIDENCE / name).read_bytes()).hexdigest() == expected, name
for process in psutil.process_iter(['cmdline']):
    command = process.info['cmdline'] or []
    assert not (command and command[0].startswith('gz sim'))
    assert not any('lab_effort.launch.py' in argument for argument in command if process.pid != os.getppid())
command = ('ulimit -c 0; source /opt/ros/jazzy/setup.bash; source install/setup.bash; '
           'source install_lab_scene_48box_sequential_full_20261024_r359/local_setup.bash; '
           'exec ros2 launch ' + shlex.quote(str(EVIDENCE / 'inputs/lab_dual.launch.py')) +
           ' scene:=centrifuge with_48box:=true enable_robot_press:=true allow_open_lid_demo:=false')
environment = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', ROS_LOG_DIR=str(EVIDENCE / 'run/ros_log'),
                   ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', QT_SCALE_FACTOR='1',
                   QT_AUTO_SCREEN_SCALE_FACTOR='0',
                   GZ_TRANSPORT_RCVHWM='20000', GZ_TRANSPORT_SNDHWM='20000',
                   RMW_IMPLEMENTATION='rmw_fastrtps_cpp', FASTDDS_BUILTIN_TRANSPORTS='UDPv4')
with (EVIDENCE / 'run/launch.log').open('x') as stream:
    process = subprocess.Popen(['bash', '-c', command], cwd=ROOT, env=environment,
        stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
with (EVIDENCE / 'run/launch_pid.json').open('x') as stream:
    json.dump(dict(pid=process.pid, created=psutil.Process(process.pid).create_time(), command=command), stream, indent=2)
print(process.pid)

