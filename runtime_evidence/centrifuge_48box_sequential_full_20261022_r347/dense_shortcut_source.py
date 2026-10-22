CHANGES = (
    ('    def edge(start, end):', '    def edge(start, end, resolution=.035):'),
    ('np.max(np.abs(end - start)) / .035', 'np.max(np.abs(end - start)) / resolution'),
    ('edge(vectors[first_index], vectors[last_index])',
     'edge(vectors[first_index], vectors[last_index], resolution=.002)'),
)


def restore_dense_function(source):
    for original, current in reversed(CHANGES):
        if source.count(current) != 1:
            raise ValueError('Only the three verified dense shortcut edits may be reversed')
        source = source.replace(current, original, 1)
    return source


def restore_dense_worker_source(name, source):
    if name != 'dual_worker.py':
        return source
    imported = 'from dense_shortcut import select_free_path\n'
    current = '        samples = select_free_path(self.request, free_path)(self.planner, first, last)\n'
    original = '        samples = free_path(self.planner, first, last)\n'
    if source.count(imported) != 1 or source.count(current) != 1:
        raise ValueError('Only the declared retrieval selector seam may be reversed')
    return source.replace(imported, '', 1).replace(current, original, 1)
