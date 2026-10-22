import math
import re
import time


class WallClockAcceptance:
    def __init__(self, deadline_seconds=180., clock=time.monotonic_ns):
        if not math.isfinite(deadline_seconds) or deadline_seconds <= 0:
            raise ValueError('Positive finite wall-clock deadline required')
        self.deadline_seconds = deadline_seconds
        self.clock = clock
        self.started_ns = clock()
        self.finished = False

    def finish(self, complete_physics, complete_two_box_return):
        if self.finished:
            raise ValueError('A task interval cannot be restarted or finalized twice')
        self.finished = True
        elapsed = (self.clock() - self.started_ns) / 1e9
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError('Monotonic task time reversed')
        passed = complete_physics and complete_two_box_return and elapsed <= self.deadline_seconds
        return dict(status='PASS' if passed else 'FAIL', elapsed_wall_seconds=elapsed,
                    deadline_wall_seconds=self.deadline_seconds,
                    clock='monotonic_ns', includes_all_in_task_planning_and_waits=True,
                    simulation_clock_or_video_playback_not_used=True)


class RealSteadyRotation:
    def __init__(self):
        self.first_wall_ns = None
        self.first_sim = None

    def observe(self, wall_ns, sim, rpm, closed):
        if not all(math.isfinite(value) for value in (wall_ns, sim, rpm)):
            raise ValueError('Finite original clocks and rotor observation required')
        if not closed or not 29.8 <= rpm <= 30.2:
            self.first_wall_ns = self.first_sim = None
            return False
        if self.first_wall_ns is None:
            self.first_wall_ns, self.first_sim = wall_ns, sim
        if wall_ns < self.first_wall_ns or sim < self.first_sim:
            raise ValueError('Rotor observation clock reversed')
        return (wall_ns - self.first_wall_ns >= 30_000_000_000 and sim - self.first_sim >= 30.)


def passive_rate(first, last):
    wall_seconds = (last['wall_ns'] - first['wall_ns']) / 1e9
    sim_seconds = last['sim_seconds'] - first['sim_seconds']
    if not all(math.isfinite(value) for value in (wall_seconds, sim_seconds)) or wall_seconds <= 0 or sim_seconds < 0:
        raise ValueError('Positive actual wall interval and nonreversing simulation required')
    return dict(wall_seconds=wall_seconds, sim_seconds=sim_seconds,
                measured_sim_per_wall=sim_seconds / wall_seconds,
                full_task_or_loaded_grip_certified=False)


def controllers_active(output, required):
    output = re.sub(r'\x1b\[[0-9;]*m', '', output)
    rows = [line.split() for line in output.splitlines()]
    return all(any(len(row) >= 3 and row[0] == name and row[-1] == 'active' for row in rows)
               for name in required)
