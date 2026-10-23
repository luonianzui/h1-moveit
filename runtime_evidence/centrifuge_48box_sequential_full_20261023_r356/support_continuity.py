class SupportContinuity:
    def __init__(self, maximum_zero_force_s):
        self.maximum_zero_force_s = maximum_zero_force_s
        self.last_positive_sim = None
        self.last_sim = None
        self.maximum_gap_s = 0.0

    def update(self, sim, metrics):
        forward = self.last_sim is None or sim >= self.last_sim
        self.last_sim = sim
        if metrics['supported'] and forward:
            self.last_positive_sim = sim
        gap = float('inf') if self.last_positive_sim is None else sim - self.last_positive_sim
        self.maximum_gap_s = max(self.maximum_gap_s, gap) if self.last_positive_sim is not None else self.maximum_gap_s
        geometry_valid = (metrics['contacts_valid'] and metrics['sidewall_certified'] and
                          metrics['vertical_inside'])
        return dict(valid=bool(forward and geometry_valid and 0 <= gap <= self.maximum_zero_force_s),
                    zero_force_gap_sim_s=gap, maximum_zero_force_gap_sim_s=self.maximum_gap_s,
                    limit_sim_s=self.maximum_zero_force_s)

