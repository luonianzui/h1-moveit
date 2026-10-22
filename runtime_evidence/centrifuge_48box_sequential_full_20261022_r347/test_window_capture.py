from PIL import Image
import pytest

from window_capture import fit_frame


@pytest.mark.parametrize('dimensions', [(1200, 900), (3676, 2022), (1601, 1101), (600, 450), (800, 1500)])
def test_all_corners_preserved_after_window_resize(dimensions):
    source = Image.new('RGB', dimensions)
    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 0, 255)]
    markers = [(0, 0), (dimensions[0] - 80, 0), (0, dimensions[1] - 80),
               (dimensions[0] - 80, dimensions[1] - 80)]
    for color, (left, top) in zip(colors, markers):
        source.paste(color, (left, top, left + 80, top + 80))
    frame, geometry = fit_frame(source)
    assert frame.size == (1200, 900)
    assert geometry['entire_window']
    pixels = set(frame.getdata())
    assert all(color in pixels for color in colors)
