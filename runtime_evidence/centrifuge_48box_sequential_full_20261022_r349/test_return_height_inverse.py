ORIGINAL = "table[:3, 3] = [.44, .38 if request['hand'] == 'left' else -.38, 1.14475]"
CURRENT = "table[:3, 3] = [.44, .38 if request['hand'] == 'left' else -.38, 1.13475 if request['hand'] == 'right' else 1.14475]"

def undo_return_height(source):
    from test_pilot_leveling_inverse import undo_pilot_leveling
    source = undo_pilot_leveling(source)
    assert source.count(CURRENT)==1
    return source.replace(CURRENT,ORIGINAL,1)
