import inspect

from execute_cycle import FullCycle
from retrieval_worker import RetrievalPlanner


def test_full_sequence_is_not_preplaced_single_extraction():
    source = inspect.getsource(FullCycle.run_complete)
    assert source.index('self.prepare_open()') < source.index('super().run()') < source.index('self.machine_cycle()') < source.index('self.retrieve()')
    source = inspect.getsource(FullCycle.retrieve)
    assert "sorted(('left', 'right')" in source
    assert "['lid', 'lid', 'rotor', 'rotor', 'lid']" in source
    assert 'two_box_load_close_spin_open_retrieve' in source


def test_original_motor_cap_and_reference_health_remain_required():
    source = inspect.getsource(FullCycle.safety)
    assert "stopped_velocity_gain') != 80" in source
    assert "stopped_position_gain') != 8000" in source
    assert '2.000001' in source
    assert 'Encoder parking reference missing during retrieval' in source


def test_known_failed_local_axis_pilot_is_not_default():
    source = inspect.getsource(RetrievalPlanner.transport)
    assert 'relative = np.linalg.inv(bucket) @ body' in source
    assert 'pilot_body[:3, 3] -= bucket[:3, 0] * relative[0, 3]' in source
    assert 'self.checked_path(pilot, basket_start=True, carry_reference=reference)' in source

