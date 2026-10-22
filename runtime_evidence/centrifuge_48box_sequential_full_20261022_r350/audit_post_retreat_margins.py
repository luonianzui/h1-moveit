import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as element_tree
import math
from types import SimpleNamespace

import numpy as np

from gripper_return_geometry import verify_return_margins


ROOT = Path(__file__).resolve().parent
source = ROOT / 'post_retreat_gripper_probe.json'
samples = json.loads(source.read_text())['remaining']['stages'][0]['samples']
names = list(samples[0])
tree = element_tree.parse(ROOT / 'inputs/robot_dual.urdf').getroot()
limits = [tree.find(f"./joint[@name='{name}']/limit") for name in names]
lower = np.array([float(limit.get('lower')) + .02 for limit in limits])
upper = np.array([float(limit.get('upper')) - .02 for limit in limits])
torso = names.index('torso_joint')
lower[torso], upper[torso] = math.radians(-44), math.radians(44)
verify_return_margins(SimpleNamespace(names=names, lower=lower, upper=upper), samples)
with (ROOT / 'post_retreat_margin_audit.json').open('x') as stream:
    json.dump(dict(status='PASS', source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                   samples=len(samples), original_revolute_margin_rad=.02, original_torso_margin_deg=44.,
                   no_fresh_physics_or_180_claim=True), stream, indent=2)
print('ORIGINAL_POST_RETREAT_JOINT_MARGINS_PASS', len(samples), flush=True)
