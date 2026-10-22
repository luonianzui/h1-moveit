"""Source contracts for the read-only Gazebo handle-contact authority."""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET


WORKSPACE = Path(__file__).resolve().parents[3]
WORLD = WORKSPACE / "src/h1_2_description/worlds/h1_world.sdf"
MODEL = WORKSPACE / "src/h1_2_description/models/incubator/model.sdf"
LAUNCH = WORKSPACE / "src/h1_2_moveit_config/launch/gazebo.launch.py"


def test_world_loads_contact_system_once():
    root = ET.parse(WORLD).getroot()
    plugins = [
        plugin for plugin in root.findall(".//world/plugin")
        if plugin.attrib.get("name") == "gz::sim::systems::Contact"
    ]
    assert len(plugins) == 1
    assert plugins[0].attrib["filename"] == "gz-sim-contact-system"


def test_handle_sensor_is_observational_and_scoped_to_original_collision():
    root = ET.parse(MODEL).getroot()
    sensors = root.findall(".//link[@name='handle_link']/sensor")
    assert len(sensors) == 1
    sensor = sensors[0]
    assert sensor.attrib == {"name": "handle_contact_829", "type": "contact"}
    assert sensor.findtext("always_on") == "true"
    assert sensor.findtext("update_rate") == "1000"
    assert sensor.findtext("contact/collision") == "handle_collision"
    assert sensor.findtext("contact/topic") == "/incubator/handle/contacts"
    assert root.find(".//link[@name='handle_link']/collision[@name='handle_collision']") is not None


def test_launch_has_one_way_contacts_bridge_and_no_contact_command_bridge():
    source = LAUNCH.read_text(encoding="utf-8")
    strings = [
        node.value for node in ast.walk(ast.parse(source))
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    ]
    binding = (
        "/incubator/handle/contacts@"
        "ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"
    )
    assert strings.count(binding) == 1
    assert not any("ros_gz_interfaces/msg/Contacts]gz.msgs.Contacts" in value
                   for value in strings)
    assert "incubator_handle_contact_bridge" in source
