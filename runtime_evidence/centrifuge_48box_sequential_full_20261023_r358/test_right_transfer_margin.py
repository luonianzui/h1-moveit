from pathlib import Path

import pytest

from pipeline_source import restore_pipeline_source


ROOT = Path(__file__).resolve().parent
PARENT = ROOT.parent / 'centrifuge_48box_sequential_full_20261021_r334'


def test_exactly_one_intermediate_height_changed():
    name = 'speed_retrieval_geometry.py'
    source = (ROOT / name).read_text()
    assert restore_pipeline_source(name, source) == (PARENT / name).read_text().replace(PARENT.name, ROOT.name)
    assert source.count('down[2, 3] = .84475') == 1
    assert (ROOT / 'sequential_geometry.py').read_bytes() == (PARENT / 'sequential_geometry.py').read_bytes()


@pytest.mark.parametrize('change', ['1.12375', '1.13475', '1.14475'])
def test_undeclared_transfer_height_is_rejected(change):
    source = (ROOT / 'speed_retrieval_geometry.py').read_text().replace('1.12475 if', change + ' if')
    with pytest.raises(ValueError):
        restore_pipeline_source('speed_retrieval_geometry.py', source)
