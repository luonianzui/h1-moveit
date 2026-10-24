IMPORTED = 'from elbow_transport_rate import RETRIEVAL_TRANSPORT_PHASES, production_retrieval_transport_rate\n'
CONSTRUCTOR_LINE = '        self.elbow_retrieval_transport_cap = .3\n'
METHOD = '''    def free_transport_rate(self, phase, requested):
        inherited = super().free_transport_rate(phase, requested)
        if phase in RETRIEVAL_TRANSPORT_PHASES and getattr(self, 'elbow_retrieval_transport_cap', None) is not None:
            return production_retrieval_transport_rate(self, phase, inherited)
        return inherited

'''


def restore_elbow_transport(name, source):
    if name == 'endpoint_overlap_cycle.py' and IMPORTED in source:
        assert source.count(IMPORTED) == source.count(CONSTRUCTOR_LINE) == source.count(METHOD) == 1
        return source.replace(IMPORTED, '', 1).replace(CONSTRUCTOR_LINE, '', 1).replace(METHOD, '', 1)
    return source
