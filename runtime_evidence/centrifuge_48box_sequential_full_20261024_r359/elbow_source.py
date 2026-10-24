IMPORT = 'from elbow_posture import ELBOW_CONTRACT, refine, posture_metrics\n'
METHOD = "    def solve(self, bodies, seed):\n        return refine(self, bodies, seed, super().solve)\n\n"
MARKER = "    request['elbow_posture_contract'] = ELBOW_CONTRACT\n"
CHOICE = "    from elbow_posture import active, choose_elbow_pickup\n    if active(request):\n        return choose_elbow_pickup(request, worker)\n"
POSTURE = "        summary = {hand: max(posture_metrics(planner, point)[hand]['from_down_deg'] for point in carry['stages'][0 if request['retrieval'] else -1]['samples']) for hand in ('left', 'right')}\n        elbow_summary = dict(basket_max_from_down_deg=max(summary.values()), by_hand_from_down_deg=summary)\n"


def restore_elbow(name, source):
    if name == 'dual_worker.py' and IMPORT in source:
        assert source.count(IMPORT) == source.count(METHOD) == source.count(MARKER) == 1
        source = source.replace(IMPORT, '', 1).replace(METHOD, '', 1).replace(MARKER, '', 1)
    if name == 'minimum_turn.py' and CHOICE in source:
        assert source.count(CHOICE) == 1
        source = source.replace(CHOICE, '', 1)
        source = source.replace('    from elbow_posture import active, posture_metrics\n', '', 1)
        source = source.replace('    elbow_summary = None\n    if active(request):\n' + POSTURE, '', 1)
        source = source.replace('                elbow_posture=elbow_summary,\n', '', 1)
    return source
