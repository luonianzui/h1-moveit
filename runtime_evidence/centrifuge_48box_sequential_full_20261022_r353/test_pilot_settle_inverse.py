def undo_pilot_settle_integration(source):
    from test_single_setdown_inverse import undo_single_setdown_integration
    source = undo_single_setdown_integration(source)
    changes = [
        'from pilot_settle import settle_retrieval_pilot\n',
        '            settle_retrieval_pilot(self, hand, payload)\n']
    for change in changes:
        assert source.count(change) == 1
        source = source.replace(change, '', 1)
    return source
