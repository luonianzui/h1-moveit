import math


def motion_wall_budget(duration_sim_s):
    if not math.isfinite(duration_sim_s) or duration_sim_s <= 0:
        raise ValueError('Positive finite actual command duration required')
    return min(900., max(60., (duration_sim_s + .45) * 50.))


class SimulationProgressWatch:
    def __init__(self, duration_sim_s, start_sim, start_wall):
        if not all(math.isfinite(value) and value >= 0 for value in (start_sim, start_wall)):
            raise ValueError('Finite actual source clocks required')
        self.budget_wall_s = motion_wall_budget(duration_sim_s)
        self.start_wall = start_wall
        self.last_wall = start_wall
        self.last_sim = start_sim
        self.last_advance_wall = start_wall

    def observe(self, source_sim, now_wall):
        if not all(math.isfinite(value) for value in (source_sim, now_wall)):
            raise ValueError('Nonfinite source clock')
        if source_sim < self.last_sim - 1e-9 or now_wall < self.last_wall:
            raise ValueError('Actual source clock reversal')
        if source_sim > self.last_sim + 1e-12:
            self.last_sim = source_sim
            self.last_advance_wall = now_wall
        elif now_wall - self.last_advance_wall > 2.:
            raise TimeoutError('Actual simulation clock stopped advancing for two wall seconds')
        self.last_wall = now_wall
        if now_wall - self.start_wall > self.budget_wall_s:
            raise TimeoutError('Paired finite wall budget exceeded')


def original_wall_budget(duration_sim_s):
    if not math.isfinite(duration_sim_s) or duration_sim_s <= 0:
        raise ValueError('Positive finite actual command duration required')
    return max(60., duration_sim_s * 25.)
