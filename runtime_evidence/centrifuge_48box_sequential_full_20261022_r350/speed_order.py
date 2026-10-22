from sequential_order import SequentialOrder, finite_duration
from speed_policy import PROFILE


class SpeedPriorityOrder(SequentialOrder):
    def finish_loading(self, hand, payload, actual_holders, stable_drop_sim_s):
        if self.phase != 'loading' or self.holder != dict(hand=hand, payload=payload) or actual_holders:
            raise ValueError('Actual single-box release required')
        finite_duration(stable_drop_sim_s, PROFILE['release_hold_sim_s'])
        self.locations[payload] = 'basket'
        self.loaded.append(payload)
        self.holder = None
        if len(self.loaded) == 2:
            self.phase = 'machine'

    def finish_return(self, hand, payload, actual_holders, stable_table_sim_s):
        if self.phase != 'retrieval' or self.holder != dict(hand=hand, payload=payload) or actual_holders:
            raise ValueError('Actual supported table release required')
        finite_duration(stable_table_sim_s, PROFILE['release_hold_sim_s'])
        self.locations[payload] = 'table'
        self.returned.append(hand)
        self.holder = None
        if len(self.returned) == 2:
            self.phase = 'terminal'

    def finish_terminal(self, actual_holders, stable_table_sim_s):
        if self.phase != 'terminal' or actual_holders or self.holder is not None:
            raise ValueError('Both actual empty hands required')
        finite_duration(stable_table_sim_s, PROFILE['terminal_hold_sim_s'])
        if self.locations != dict(left='table', right='table'):
            raise ValueError('Both distinct boxes must return to the table')
        self.phase = 'complete'
