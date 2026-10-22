IMPORTED = 'from micro_alignment import alignment_stage_parameters\n'
ANCHOR = '        samples = [dict(seed)]\n'
INSERTED = '        count, guides = alignment_stage_parameters(self.request, phase, first, last, count, guides)\n'


def add_micro_alignment(source):
    anchor = 'from lift_target_provider import stage_body_targets\n'
    assert source.count(anchor)==source.count(ANCHOR)==1
    return source.replace(anchor,anchor+IMPORTED,1).replace(ANCHOR,INSERTED+ANCHOR,1)


def restore_micro_alignment(source):
    assert source.count(IMPORTED)==source.count(INSERTED+ANCHOR)==1
    return source.replace(IMPORTED,'',1).replace(INSERTED+ANCHOR,ANCHOR,1)
