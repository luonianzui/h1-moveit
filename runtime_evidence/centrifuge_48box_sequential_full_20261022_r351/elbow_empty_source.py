IMPORTED = 'from elbow_empty_approach import production_empty_approach_limits\n'
CONSTRUCTOR = '''    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.elbow_empty_approach_cap = .5

'''
LIMIT = '''        if phase == 'paired_load_approach' and getattr(self, 'elbow_empty_approach_cap', None) is not None:
            return production_empty_approach_limits(self, requested_rate, super().motion_limits)
'''


def restore_elbow_empty(name, source):
    if name == 'endpoint_overlap_cycle.py' and IMPORTED in source:
        assert source.count(IMPORTED) == source.count(CONSTRUCTOR) == source.count(LIMIT) == 1
        return source.replace(IMPORTED, '', 1).replace(CONSTRUCTOR, '', 1).replace(LIMIT, '', 1)
    return source
