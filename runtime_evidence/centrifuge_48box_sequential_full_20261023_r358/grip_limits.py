import math


CARRY_ANGLE_LIMIT_DEG = 5.0
CARRY_SLIP_LIMIT_M = .002


def grip_exceeds_limits(slip_m, angle_deg):
    return (not math.isfinite(slip_m) or not math.isfinite(angle_deg) or
            slip_m < 0 or slip_m > CARRY_SLIP_LIMIT_M or abs(angle_deg) >= CARRY_ANGLE_LIMIT_DEG)
