ENDPOINT_HOOKS = '''    def paired_feedback_overlap_enabled(self):
        return False

    def paired_endpoint_stamp(self, target, not_before):
        return self.sim if all(abs(self.positions[name] - value) < .0003 for name, value in target.items()) else None

'''
LOOP_ANCHOR = "        while self.sim < start + timing['duration_sim_s'] + .15:\n"
ORIGINAL_ENDPOINT = '''        stable, begin = None, self.sim
        while self.sim - begin < 5:
            self.spin()
            if all(abs(self.positions[name] - value) < .0003 for name, value in samples[-1].items()):
                stable = self.sim if stable is None else stable
                if self.sim - stable > .2:
                    return
            else:
                stable = None
'''
OVERLAPPED_ENDPOINT = '''            if self.paired_feedback_overlap_enabled():
                stamp = self.paired_endpoint_stamp(samples[-1], start + timing['duration_sim_s'])
                if stamp is None or (stable is not None and stamp < stable):
                    stable = None
                elif stable is None:
                    stable = stamp
        begin = self.sim
        while self.sim - begin < 5:
            self.spin()
            stamp = self.paired_endpoint_stamp(samples[-1], start + timing['duration_sim_s'])
            if stamp is not None:
                stable = stamp if stable is None else stable
                if stamp - stable > .2:
                    return
            else:
                stable = None
'''


def add_endpoint_overlap_hooks(source):
    anchor = '    def common_timing(self, actual, samples, rate):\n'
    assert source.count(anchor) == source.count(LOOP_ANCHOR) == source.count(ORIGINAL_ENDPOINT) == 1
    return source.replace(anchor, ENDPOINT_HOOKS + anchor, 1).replace(
        LOOP_ANCHOR, '        stable = None\n' + LOOP_ANCHOR, 1).replace(ORIGINAL_ENDPOINT, OVERLAPPED_ENDPOINT, 1)


def restore_endpoint_overlap_source(source):
    assert source.count(ENDPOINT_HOOKS) == source.count(OVERLAPPED_ENDPOINT) == 1
    assert source.count('        stable = None\n' + LOOP_ANCHOR) == 1
    return source.replace(ENDPOINT_HOOKS, '', 1).replace(
        '        stable = None\n' + LOOP_ANCHOR, LOOP_ANCHOR, 1).replace(OVERLAPPED_ENDPOINT, ORIGINAL_ENDPOINT, 1)
