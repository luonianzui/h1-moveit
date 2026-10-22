import copy
import math


HANDS = ('left', 'right')


def finite_duration(value, minimum):
    if not isinstance(value, (float, int)) or not math.isfinite(value) or value < minimum:
        raise ValueError('Original observation duration is required')


class SequentialOrder:
    def __init__(self):
        self.locations = {payload: 'table' for payload in HANDS}
        self.loaded = []
        self.returned = []
        self.holder = None
        self.phase = 'loading'
        self.retrieval_assignment = None

    def begin(self, hand, payload, actual_holders):
        if self.holder is not None or hand not in HANDS or payload not in HANDS:
            raise ValueError('Exactly one known actual holder is allowed')
        if set(actual_holders) != {hand} or actual_holders[hand].get('payload') != payload:
            raise ValueError('One matching actual holder registry entry is required')
        if self.phase == 'loading':
            expected = HANDS[len(self.loaded)] if len(self.loaded) < 2 else None
            if hand != expected or payload != hand or self.locations[payload] != 'table':
                raise ValueError('Loading must process both table boxes in left/right order')
        elif self.phase == 'retrieval':
            expected = HANDS[len(self.returned)] if len(self.returned) < 2 else None
            if (hand != expected or self.retrieval_assignment.get(hand) != payload
                    or self.locations[payload] != 'basket'):
                raise ValueError('Retrieval must respect the fresh post-spin assignment')
        else:
            raise ValueError('A machine or completed phase cannot acquire a box')
        self.holder = dict(hand=hand, payload=payload)
        self.locations[payload] = 'held'

    def finish_loading(self, hand, payload, actual_holders, stable_drop_sim_s):
        if self.phase != 'loading' or self.holder != dict(hand=hand, payload=payload) or actual_holders:
            raise ValueError('Qualified normal single-box release must precede drop acceptance')
        finite_duration(stable_drop_sim_s, 3.)
        self.locations[payload] = 'basket'
        self.loaded.append(payload)
        self.holder = None
        if len(self.loaded) == 2:
            self.phase = 'machine'

    def finish_machine(self, actual_holders, steady_spin_sim_s, machine, assignment):
        if self.phase != 'machine' or actual_holders or self.holder is not None:
            raise ValueError('Both released boxes and empty hands are required before machine completion')
        if self.locations != dict(left='basket', right='basket'):
            raise ValueError('Both boxes must remain in their own baskets')
        finite_duration(steady_spin_sim_s, 30.)
        rpm = machine.get('rotor_rpm')
        angle = machine.get('lid_rad')
        velocity = machine.get('rotor_rad_s')
        if (not isinstance(rpm, (int, float)) or not math.isfinite(rpm) or abs(rpm) > .02
                or machine.get('rotor_enabled') is not False or machine.get('can_open') is not True
                or not isinstance(velocity, (int, float)) or not math.isfinite(velocity) or abs(velocity) >= .02
                or not isinstance(angle, (int, float)) or not math.isfinite(angle)
                or abs(angle - math.radians(120)) >= math.radians(.2)
                or machine.get('starts') != 1 or machine.get('stops') != 1):
            raise ValueError('Original actual stopped/open machine gates are required')
        if set(assignment) != set(HANDS) or set(assignment.values()) != set(HANDS):
            raise ValueError('Fresh post-spin assignment must cover both distinct boxes')
        self.retrieval_assignment = dict(assignment)
        self.phase = 'retrieval'

    def finish_return(self, hand, payload, actual_holders, stable_table_sim_s):
        if self.phase != 'retrieval' or self.holder != dict(hand=hand, payload=payload) or actual_holders:
            raise ValueError('Qualified normal table release is required')
        finite_duration(stable_table_sim_s, 3.)
        self.locations[payload] = 'table'
        self.returned.append(hand)
        self.holder = None
        if len(self.returned) == 2:
            self.phase = 'terminal'

    def finish_terminal(self, actual_holders, stable_table_sim_s):
        if self.phase != 'terminal' or actual_holders or self.holder is not None:
            raise ValueError('Both completed returns and empty hands are required')
        finite_duration(stable_table_sim_s, 10.)
        if self.locations != dict(left='table', right='table'):
            raise ValueError('Both boxes must be supported on the table')
        self.phase = 'complete'

    def snapshot(self):
        return copy.deepcopy(dict(phase=self.phase, locations=self.locations, loaded=self.loaded,
                                  returned=self.returned, holder=self.holder,
                                  retrieval_assignment=self.retrieval_assignment))
