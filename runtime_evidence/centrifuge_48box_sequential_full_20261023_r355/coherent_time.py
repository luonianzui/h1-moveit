from bisect import bisect_left
from collections import deque
import math


class JointTimeline:
    def __init__(self, names, maximum_gap_s=.012, capacity=256):
        self.names = frozenset(names)
        self.maximum_gap_s = maximum_gap_s
        self.rows = deque(maxlen=capacity)
        if not self.names or not 0 < maximum_gap_s <= .012 or capacity < 2:
            raise ValueError('Invalid bounded real-joint timeline')

    def add(self, stamp, positions):
        values = {name:float(value) for name, value in positions.items()}
        if not math.isfinite(stamp) or stamp < 0 or values.keys() != self.names or not all(
                math.isfinite(value) for value in values.values()):
            raise ValueError('Incomplete or nonfinite actual joint frame')
        if self.rows and stamp <= self.rows[-1][0]:
            if stamp == self.rows[-1][0] and values == self.rows[-1][1]:
                return
            raise ValueError('Joint time reversal or conflicting duplicate')
        self.rows.append((float(stamp), values))

    def at(self, stamp):
        if not math.isfinite(stamp):
            raise ValueError('Nonfinite requested pose stamp')
        rows = list(self.rows)
        stamps = [row[0] for row in rows]
        index = bisect_left(stamps, stamp)
        if index < len(rows) and abs(stamps[index] - stamp) < 1e-10:
            return dict(positions=dict(rows[index][1]), lower_stamp=stamps[index],
                        upper_stamp=stamps[index], fraction=0., interpolation=False)
        if index == 0 or index == len(rows):
            return None
        lower, upper = rows[index - 1], rows[index]
        gap = upper[0] - lower[0]
        if gap > self.maximum_gap_s + 1e-9:
            return None
        fraction = (stamp - lower[0]) / gap
        return dict(positions={name:lower[1][name] + fraction * (upper[1][name] - lower[1][name])
                    for name in self.names}, lower_stamp=lower[0], upper_stamp=upper[0],
                    fraction=fraction, interpolation=True)


def newest_pair(frames, timeline, latest_native_stamp, maximum_lag_s=.012):
    if not 0 < maximum_lag_s <= .012:
        raise ValueError('Cannot loosen coherent-to-latest native lag')
    for stamp, wall, poses in reversed(list(frames)):
        if stamp > latest_native_stamp:
            continue
        if latest_native_stamp - stamp > maximum_lag_s + 1e-9:
            break
        measured = timeline.at(stamp)
        if measured is not None:
            return dict(pose_stamp=stamp, pose_wall=wall, poses=poses, joints=measured,
                        native_lag_s=latest_native_stamp - stamp)
    return None
