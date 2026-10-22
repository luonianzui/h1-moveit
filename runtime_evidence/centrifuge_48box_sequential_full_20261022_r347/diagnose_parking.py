import json
import math

from sequential_geometry import EVIDENCE, SinglePlanner


data = json.loads((EVIDENCE / 'safe_parking_probe.json').read_text())
planner = SinglePlanner(data['request'])
first = {name: data['request']['positions'][name] for name in planner.names}
for label, joints in [('initial', first), ('park_zero', data['records'][0]['joints']),
                      ('park_first_sweep', dict(data['records'][0]['joints'], torso_joint=-math.radians(44)))]:
    print('PARK_DIAGNOSTIC', label, flush=True)
    print(planner.planner.check(joints, verbose=True), flush=True)
