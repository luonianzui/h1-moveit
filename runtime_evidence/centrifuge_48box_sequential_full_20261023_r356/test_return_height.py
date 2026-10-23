import ast
from pathlib import Path

import pytest
from test_return_height_inverse import CURRENT, undo_return_height

ROOT=Path(__file__).resolve().parent
PARENT=ROOT.parent/'centrifuge_48box_sequential_full_20261020_r316'

def test_only_return_transfer_waypoint_height_changes_parent():
    assert undo_return_height((ROOT/'sequential_planning.py').read_text())==(PARENT/'sequential_planning.py').read_text()
    for name in ('sequential_cycle.py','sequential_geometry.py','single_setdown.py','execute_dual.py','sequential_worker.py'):
        from acceleration_source import restore_acceleration_bytes
        assert restore_acceleration_bytes(name, (ROOT/name).read_bytes())==(PARENT/name).read_bytes()

@pytest.mark.parametrize('hand,expected_y,expected_z',[('left',.38,1.14475),('right',-.38,1.13475)])
def test_only_right_height_changes_not_table_slot_xy(hand,expected_y,expected_z):
    expression=ast.parse(CURRENT).body[0].value
    target=eval(compile(ast.Expression(expression),'<tested_actual_waypoint_expression>','eval'),{'request':{'hand':hand}})
    assert target==[.44,expected_y,expected_z]

def test_runtime_guards_and_final_table_z_unchanged():
    source=(ROOT/'sequential_planning.py').read_text()
    assert source.count('down[2, 3] = .84475')==1
    assert (ROOT/'sequential_geometry.py').read_bytes()==(PARENT/'sequential_geometry.py').read_bytes()
