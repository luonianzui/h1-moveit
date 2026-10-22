import math


FREE_CARRY_PHASES=frozenset(side+suffix for side in ('left','right')
                          for suffix in ('_transfer','_retrieval_transfer'))
ORIGINAL_RATES={'left_transfer':.065,'right_transfer':.13,
                'left_retrieval_transfer':.10,'right_retrieval_transfer':.10}
MAXIMUM_RATE_RAD_S=.36
FREE_CARRY_SCALE=1.15
OBSERVED_LAG_S=.035
ORIGINAL_FLOOR_S=.020
FREE_CARRY_FLOOR_S=.020


def certified_free_carry(phase,holding,held_payload,efforts):
    if phase not in FREE_CARRY_PHASES or holding not in ('left','right') or not phase.startswith(holding+'_'):
        return False
    effort=efforts.get(holding)
    if effort is None or not math.isfinite(effort) or effort<=0:
        return False
    if '_retrieval_' in phase:
        return held_payload in ('left','right')
    return held_payload is None


def carry_rate(phase,original,holding,held_payload,efforts):
    if not math.isfinite(original) or original<=0:
        raise ValueError('Finite positive rate required')
    if not certified_free_carry(phase,holding,held_payload,efforts):
        return original
    if abs(original-ORIGINAL_RATES[phase])>1e-12:
        raise ValueError('Unreviewed original free-carry profile')
    proposed=min(original*FREE_CARRY_SCALE,MAXIMUM_RATE_RAD_S)
    assert proposed*OBSERVED_LAG_S<=.0126+1e-12
    return proposed


def carry_floor(phase,holding,held_payload,efforts):
    return FREE_CARRY_FLOOR_S if certified_free_carry(phase,holding,held_payload,efforts) else ORIGINAL_FLOOR_S
