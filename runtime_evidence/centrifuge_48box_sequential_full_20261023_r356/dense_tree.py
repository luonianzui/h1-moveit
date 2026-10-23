import time

import numpy as np


def free_path(planner, first, last):
    names = list(first)
    origin = np.array([first[name] for name in names])
    goal = np.array([last[name] for name in names])
    generator = np.random.default_rng(480121)
    checks = 0

    def valid(vector):
        nonlocal checks
        checks += 1
        return not planner.check(dict(zip(names, vector)))['collision']

    def edge(start, end, resolution=.002):
        count = max(2, int(np.ceil(np.max(np.abs(end - start)) / resolution)) + 1)
        return all(valid(vector) for vector in np.linspace(start, end, count))

    if not valid(origin) or not valid(goal):
        raise ValueError('RRT endpoint in collision')
    trees = [[(origin, None)], [(goal, None)]]
    lower = np.array([float(planner.urdf.find(f"./joint[@name='{name}']/limit").get('lower')) + .02 for name in names])
    upper = np.array([float(planner.urdf.find(f"./joint[@name='{name}']/limit").get('upper')) - .02 for name in names])
    lower[0], upper[0] = np.deg2rad([-40, 40])

    def extend(tree, target):
        index = int(np.argmin([np.linalg.norm(node[0] - target) for node in tree]))
        start = tree[index][0]
        distance = np.linalg.norm(target - start)
        if distance < 1e-9:
            return index, True
        end = start + min(1, .25 / distance) * (target - start)
        if not edge(start, end):
            return None, False
        tree.append((end, index))
        return len(tree) - 1, distance <= .25

    def trace(tree, index):
        path = []
        while index is not None:
            point, index = tree[index]
            path.append(point)
        return path[::-1]

    deadline = time.monotonic() + 120
    for iteration in range(2000):
        if time.monotonic() > deadline:
            break
        active = iteration % 2
        if iteration % 5 == 0:
            target = trees[1 - active][0][0]
        elif iteration % 3:
            target = np.clip(origin + generator.random() * (goal - origin) + generator.normal(0, .65, len(names)), lower, upper)
        else:
            target = generator.uniform(lower, upper)
        index, reached = extend(trees[active], target)
        if index is None:
            continue
        meeting = trees[active][index][0]
        for attempt in range(60):
            other_index, reached = extend(trees[1 - active], meeting)
            if other_index is None:
                break
            if reached:
                paths = [None, None]
                paths[active] = trace(trees[active], index)
                paths[1 - active] = trace(trees[1 - active], other_index)
                vectors = paths[0] + paths[1][::-1][1:]
                for attempt in range(40):
                    first_index, last_index = sorted(generator.choice(len(vectors), 2, replace=False))
                    if last_index > first_index + 1 and edge(vectors[first_index], vectors[last_index], resolution=.002):
                        vectors = vectors[:first_index + 1] + vectors[last_index:]
                samples = []
                for start, end in zip(vectors, vectors[1:]):
                    count = max(2, int(np.ceil(np.max(np.abs(end - start)) / .02)) + 1)
                    samples.extend(dict(zip(names, point.tolist())) for point in np.linspace(start, end, count)[1:])
                print('RRT found', iteration, checks, len(samples), flush=True)
                return [first] + samples
    raise ValueError(f'RRT no path after {checks} collision queries')


def select_free_path(request, legacy):
    if request.get('mode') != 'retrieval_pickup':
        return legacy
    if request.get('retrieval') is not True:
        raise ValueError('Dense shortcut requires the declared actual retrieval pickup scope')
    if request.get('original_runtime_holder_references') != {}:
        raise ValueError('Empty retrieval pickup requires explicitly empty actual holder references')
    return free_path

