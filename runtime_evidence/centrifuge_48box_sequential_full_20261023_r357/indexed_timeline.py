from bisect import bisect_left
import math
from operator import itemgetter

from coherent_time import JointTimeline


STAMP = itemgetter(0)


class IndexedJointTimeline(JointTimeline):
    def at(self, stamp):
        if not math.isfinite(stamp):
            raise ValueError('Nonfinite requested pose stamp')
        rows = self.rows
        index = bisect_left(rows, stamp, key=STAMP)
        if index < len(rows) and abs(rows[index][0] - stamp) < 1e-10:
            matched = rows[index]
            return dict(positions=dict(matched[1]), lower_stamp=matched[0],
                        upper_stamp=matched[0], fraction=0., interpolation=False)
        if index == 0 or index == len(rows):
            return None
        lower, upper = rows[index - 1], rows[index]
        gap = upper[0] - lower[0]
        if gap > self.maximum_gap_s + 1e-9:
            return None
        fraction = (stamp - lower[0]) / gap
        return dict(positions={name: lower[1][name] + fraction * (upper[1][name] - lower[1][name])
                    for name in self.names}, lower_stamp=lower[0], upper_stamp=upper[0],
                    fraction=fraction, interpolation=True)


def indexed_copy(original):
    candidate = IndexedJointTimeline(original.names, original.maximum_gap_s, original.rows.maxlen)
    for stamp, positions in original.rows:
        candidate.add(stamp, positions)
    return candidate
