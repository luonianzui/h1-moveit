from pathlib import Path
import runpy

import pytest


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261022_r352'
RELAY = runpy.run_path(str(ROOT / 'inputs/robot_joint_state_relay_05ms.py'))


def test_native_source_guard_is_byte_identical():
    assert (ROOT / 'inputs/robot_joint_state_relay_05ms.py').read_bytes() == (PARENT / 'inputs/robot_joint_state_relay_05ms.py').read_bytes()


@pytest.mark.parametrize('step', [0, 499999, 500001, 1000000, -500000])
def test_source_loss_is_not_hidden(step):
    core = RELAY['RelayCore']()
    core.accept_clock(1000000)
    with pytest.raises(RELAY['RelayError']):
        core.accept_clock(1000000 + step)


@pytest.mark.parametrize('name', ['launch_scene.py', 'run_dev.sh', 'start_front_recording.sh'])
def test_process_only_udp_transport_bound(name):
    text = (ROOT / name).read_text()
    assert 'FASTDDS_BUILTIN_TRANSPORTS' in text and 'UDPv4' in text and 'rmw_fastrtps_cpp' in text
    assert '/etc/' not in text


def test_telemetry_qos_and_step_unchanged():
    assert RELAY['SOURCE_STEP_NS'] == 500000
    assert RELAY['OUTPUT_PERIOD_NS'] == 10000000
    text = (ROOT / 'inputs/robot_joint_state_relay_05ms.py').read_text()
    assert 'depth=20_000' in text and 'ReliabilityPolicy.RELIABLE' in text
