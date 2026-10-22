PREPARE_HOOK = '''    def prepare_loading_grippers(self):
        for target, phase in ((-.0199, 'paired_load_open_empty'), (-.01901, 'paired_load_close_empty_check'),
                              (-.0199, 'paired_load_reopen_empty')):
            self.paired_grippers(target, 0, phase)

    def paired_gripper_post_action_dwell(self):
        return .3

'''
ORIGINAL_PREPARE = '''        for target, phase in ((-.0199, 'paired_load_open_empty'), (-.01901, 'paired_load_close_empty_check'),
                              (-.0199, 'paired_load_reopen_empty')):
            self.paired_grippers(target, 0, phase)
'''


def add_empty_protocol_hooks(source):
    anchor = '    def paired_grippers(self, value, effort, phase, retrieval=False):\n'
    assert source.count(anchor) == source.count(ORIGINAL_PREPARE) == 1
    source = source.replace(ORIGINAL_PREPARE, '        self.prepare_loading_grippers()\n', 1)
    source = source.replace(anchor, PREPARE_HOOK + anchor, 1)
    anchor = '        self.retrieval_closing = False\n        self.dwell(.3)\n'
    assert source.count(anchor) == 1
    return source.replace(anchor, '        self.retrieval_closing = False\n        self.dwell(self.paired_gripper_post_action_dwell())\n', 1)


def restore_empty_protocol_source(source):
    assert source.count(PREPARE_HOOK) == source.count('        self.prepare_loading_grippers()\n') == 1
    source = source.replace(PREPARE_HOOK, '', 1)
    source = source.replace('        self.prepare_loading_grippers()\n', ORIGINAL_PREPARE, 1)
    anchor = '        self.retrieval_closing = False\n        self.dwell(self.paired_gripper_post_action_dwell())\n'
    assert source.count(anchor) == 1
    return source.replace(anchor, '        self.retrieval_closing = False\n        self.dwell(.3)\n', 1)
