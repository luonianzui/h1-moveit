IMPORTED = 'from minimum_turn import CONTRACT, branch_seed, choose_pickup\n'
WRAPPER = '\ndef plan_pickup(request):\n    return choose_pickup(request, sys.modules[__name__])\n\n'
PREFERRED = '            preferred = seed if guides is None else guides[min(index, len(guides) - 1)]\n'
REPLACEMENT = '            preferred = branch_seed(self.request, phase, seed, guides, index)\n'
MARKER = "    request['minimum_turn_contract'] = CONTRACT\n"


def restore_minimum_turn(name, source):
    from elbow_source import restore_elbow
    source = restore_elbow(name, source)
    if name != 'dual_worker.py':
        return source
    if IMPORTED not in source:
        return source
    assert source.count(IMPORTED) == source.count(WRAPPER) == source.count(REPLACEMENT) == source.count(MARKER) == 1
    source = source.replace(IMPORTED, '', 1).replace(WRAPPER, '', 1).replace(REPLACEMENT, PREFERRED, 1)
    source = source.replace('def legacy_plan_pickup(request):', 'def plan_pickup(request):', 1)
    return source.replace(MARKER, '', 1)
