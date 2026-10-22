import hashlib
import json
from pathlib import Path


def undo_frame_integration(source):
    from test_pilot_settle_inverse import undo_pilot_settle_integration
    source = undo_pilot_settle_integration(source)
    changes = [
        'from preload_floor_frame import FloorContactHistory, single_missing_floor_frame\n',
        '        self.closing_floor_history = FloorContactHistory()\n',
        "        if kind == 'contact':\n            self.closing_floor_history.observe(payload)\n",
        "                    if not continuity['valid']:\n                        frame = single_missing_floor_frame(self, payload, metrics)\n                        metrics['single_actual_floor_frame'] = frame\n                        if frame['valid']:\n                            self.record('sequential_single_frame_floor_continuity', dict(payload=payload,\n                                        hand=active_hand, certificate=frame, metrics=metrics,\n                                        actual_holders=self.holders.snapshot()))\n                            continuity['valid'] = True\n                            continuity['support_source'] = frame['source']\n                            continuity['not_a_strong_grip_certificate'] = True\n"]
    for change in changes:
        assert source.count(change) == 1
        source = source.replace(change, '')
    return source


def test_only_declared_single_contact_frame_integration_changes_parent_cycle():
    root = Path(__file__).resolve().parent
    parent = root.parent / 'centrifuge_48box_sequential_full_20261019_r310'
    manifest = json.loads((parent / 'identity.json').read_text())['sha256']
    restored = undo_frame_integration((root / 'sequential_cycle.py').read_text())
    assert hashlib.sha256(restored.encode()).hexdigest() == manifest['sequential_cycle.py']
