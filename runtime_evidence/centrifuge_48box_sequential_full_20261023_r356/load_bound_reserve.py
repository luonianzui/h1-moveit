from sequential_geometry import SinglePlanner


LOAD_IK_RESERVE_RAD = .001


class InteriorLoadPlanner(SinglePlanner):
    def solve(self, body, seed, refine=True):
        original_lower, original_upper = self.lower, self.upper
        self.lower, self.upper = original_lower.copy(), original_upper.copy()
        self.lower[self.indices] += LOAD_IK_RESERVE_RAD
        self.upper[self.indices] -= LOAD_IK_RESERVE_RAD
        try:
            return super().solve(body, seed, refine=refine)
        finally:
            self.lower, self.upper = original_lower, original_upper
