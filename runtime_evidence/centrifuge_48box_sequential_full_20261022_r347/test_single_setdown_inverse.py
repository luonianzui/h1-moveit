IMPORT = 'from single_setdown import execute_single_setdown, single_setdown_timing\n'
METHOD = """    def common_timing(self, actual, samples, rate):
        if self.phase == 'paired_retrieval_setdown':
            return single_setdown_timing(self, actual, samples, rate)
        return super().common_timing(actual, samples, rate)

"""
CALL = '                execute_single_setdown(self, planned, samples, split)\n'
ORIGINAL = """                self.move(samples[:split + 1], 'paired_retrieval_setdown', .13)
                self.move(samples[split:], 'paired_retrieval_setdown_fine', .03)
"""

def undo_single_setdown_integration(source):
    source = source.replace('from bounded_release_alignment import ', 'from release_alignment import ')
    source = source.replace("coarse_rate_rad_s=.06 if planned['single_hand']=='right' else .13", "coarse_rate_rad_s=.13")
    assert source.count(IMPORT) == source.count(METHOD) == source.count(CALL) == 1
    return source.replace(IMPORT,'',1).replace(METHOD,'',1).replace(CALL,ORIGINAL,1)
