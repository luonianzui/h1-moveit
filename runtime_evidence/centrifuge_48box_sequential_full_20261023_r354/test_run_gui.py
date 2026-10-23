from pathlib import Path
import xml.etree.ElementTree as ET


EVIDENCE = Path(__file__).resolve().parent
ROOT = EVIDENCE.parents[1]


def normalized(element):
    return element.tag, sorted(element.attrib.items()), (element.text or '').strip(), [normalized(child) for child in element]


def read_gui(path):
    return ET.fromstring('<config>' + path.read_text().split('?>', 1)[1] + '</config>')


def test_only_pause_step_plugin_is_removed():
    original = read_gui(ROOT / 'src/h1_2_moveit_config/config/centrifuge_gui.config')
    generated = read_gui(EVIDENCE / 'inputs/centrifuge_run_gui.config')
    control = next(plugin for plugin in original.findall('plugin') if plugin.get('filename') == 'WorldControl')
    original.remove(control)
    original.find("plugin[@filename='MinimalScene']/camera_pose").text = '1.65 -1.55 1.95 0.0 0.44652461926754994 2.305390786134123'
    assert normalized(generated) == normalized(original)
    assert all(plugin.get('filename') != 'WorldControl' for plugin in generated.findall('plugin'))


def test_gui_fork_is_bound_to_unique_launch():
    source = (EVIDENCE / 'inputs/lab_effort.launch.py').read_text()
    assert str(EVIDENCE / 'inputs/centrifuge_run_gui.config') in source


def test_retrieval_starts_from_measured_neutral_without_legacy_parking():
    source = (EVIDENCE / 'execute_cycle.py').read_text()
    assert "'_retrieval_park'" not in source
    assert "'_retrieval_open_empty'" in source
    assert "positions=dict(self.positions), poses=deepcopy(self.poses)" in source

