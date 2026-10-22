FREE_EMPTY_PHASES = frozenset((
    'machine_sequential_empty_neutral',
    'seq_load_empty_park', 'seq_load_empty_approach',
    'seq_retrieval_empty_park', 'seq_retrieval_empty_approach',
))


def eligible(phase, holders):
    return not holders and phase in FREE_EMPTY_PHASES
