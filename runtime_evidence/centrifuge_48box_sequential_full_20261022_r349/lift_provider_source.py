IMPORTED = 'from lift_target_provider import stage_body_targets\n'
ORIGINAL = '        for index, bodies in enumerate(paired_interpolation(first, last, count)):\n'
CURRENT = '''        targets = stage_body_targets(self.request, phase, first, last, count, guides, paired_interpolation,
                                     ROOT / 'src/h1_2_description/models/centrifuge/model.sdf')
        for index, bodies in enumerate(targets):
'''


def add_lift_provider(source):
    anchor = 'from dense_tree import select_free_path\n'
    assert source.count(anchor) == source.count(ORIGINAL) == 1
    return source.replace(anchor, anchor + IMPORTED, 1).replace(ORIGINAL, CURRENT, 1)


def restore_lift_provider(source):
    assert source.count(IMPORTED) == source.count(CURRENT) == 1
    return source.replace(IMPORTED, '', 1).replace(CURRENT, ORIGINAL, 1)
