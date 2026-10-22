import importlib.util
from pathlib import Path

import pytest


path = Path(__file__).parents[1] / 'scripts/centrifuge_contact_button.py'
spec = importlib.util.spec_from_file_location('contact_button', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_requires_release_before_press_and_does_not_repeat():
    latch = module.PressLatch()
    assert not latch.update(0, True)
    assert not latch.update(.2, True)
    assert not latch.update(.3, False)
    assert not latch.update(.5, False)
    assert not latch.update(.6, True)
    assert not latch.update(.62, True)
    assert latch.update(.65, True)
    assert not latch.update(2, True)
    assert not latch.update(3, False)
    assert not latch.update(3.2, False)
    assert not latch.update(3.3, True)
    assert latch.update(3.4, True)


def test_time_reset_and_nonfinite_disarm():
    latch = module.PressLatch()
    latch.update(1, False)
    latch.update(2, False)
    assert latch.armed
    assert not latch.update(.5, True)
    assert not latch.armed
    assert not latch.update(float('nan'), True)


@pytest.mark.parametrize('button,hand', [('lid', 'left_dex1_finger_1'),
                                        ('rotor', 'right_dex1_finger_2')])
def test_contact_pair_filter(button, hand):
    cap = 'centrifuge::assembly::' + button + '_button_collision'
    finger = 'h1_2::' + hand + '_slider_link::' + hand + '_tip_fixed_joint_lump__collision'
    assert module.matching_contact(button, cap, finger, [.0001])
    assert module.matching_contact(button, finger, cap, [0, .0002])
    assert not module.matching_contact(button, cap, finger, [])
    assert not module.matching_contact(button, cap, finger, [.003])
    assert not module.matching_contact(button, cap, finger, [float('nan')])
    assert not module.matching_contact(button, cap, finger.replace('h1_2::', 'other::'), [.0001])
    assert not module.matching_contact(button, cap.replace('button_collision', 'button_base_collision'),
                                       finger, [.0001])
