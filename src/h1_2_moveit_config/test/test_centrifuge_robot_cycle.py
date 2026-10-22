import json
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).parents[2]


def test_sensors_watch_only_button_caps():
    model = ET.parse(ROOT / 'h1_2_description/models/centrifuge/model.sdf').getroot()
    for button in ('lid', 'rotor'):
        sensor = model.find(f"./model/link[@name='assembly']/sensor[@name='{button}_button_contact']")
        assert sensor.get('type') == 'contact'
        assert sensor.findtext('contact/collision') == button + '_button_collision'
        assert sensor.findtext('contact/topic') == '/centrifuge/' + button + '_button/contacts'
        assert sensor.find('topic') is None
        assert float(sensor.findtext('update_rate')) == 100
    assert model.find("./model/plugin[@name='CentrifugeInterlock']/allow_open_lid_demo") is None


def test_press_executor_never_fakes_button_or_lid_commands():
    source = (ROOT / 'h1_2_moveit_config/scripts/centrifuge_robot_cycle.py').read_text()
    assert '/centrifuge/button/action' not in source
    assert '/centrifuge/lid/target_angle' not in source
    assert 'Bool(data=True)' not in source
    assert '30.0' in source
