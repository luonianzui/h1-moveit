import math
import time

import numpy as np


def route_button(planner, side, first, last):
    names = list(planner.model.get_joint_model_group(side + '_arm').active_joint_model_names)
    limits = [planner.urdf.find(f"./joint[@name='{name}']/limit") for name in names]
    lower = np.array([float(limit.get('lower')) + .02 for limit in limits])
    upper = np.array([float(limit.get('upper')) - .02 for limit in limits])
    generator = np.random.default_rng(241)

    def valid(vector):
        return bool(np.all(vector >= lower) and np.all(vector <= upper) and not planner.collision(side, vector)[0])

    def edge(start, end):
        count = max(2, math.ceil(float(np.max(np.abs(end - start))) / .002) + 1)
        return all(valid(vector) for vector in np.linspace(start, end, count))

    if not valid(first) or not valid(last):
        raise ValueError('Button empty RRT endpoint collision or original joint limit')
    if edge(first, last):
        return [np.array(first), np.array(last)]
    trees = [[(np.array(first), None)], [(np.array(last), None)]]

    def extend(tree, target):
        index = int(np.argmin([np.linalg.norm(node[0] - target) for node in tree]))
        origin = tree[index][0]
        distance = float(np.linalg.norm(target - origin))
        if distance < 1e-9:
            return index, True
        point = origin + min(1., .22 / distance) * (target - origin)
        if not edge(origin, point):
            return None, False
        tree.append((point, index))
        return len(tree) - 1, distance <= .22

    def trace(tree, index):
        result = []
        while index is not None:
            point, index = tree[index]
            result.append(point)
        return result[::-1]

    deadline = time.monotonic() + 90
    for iteration in range(2400):
        if time.monotonic() > deadline:
            break
        active = iteration % 2
        if iteration % 5 == 0:
            target = trees[1 - active][0][0]
        elif iteration % 3:
            target = np.clip(first + generator.random() * (last - first) + generator.normal(0, .55, len(names)), lower, upper)
        else:
            target = generator.uniform(lower, upper)
        index, reached = extend(trees[active], target)
        if index is None:
            continue
        for attempt in range(70):
            other, reached = extend(trees[1 - active], trees[active][index][0])
            if other is None:
                break
            if reached:
                paths = [None, None]
                paths[active] = trace(trees[active], index)
                paths[1 - active] = trace(trees[1 - active], other)
                result = paths[0] + paths[1][::-1][1:]
                for shortcut in range(60):
                    start_index, end_index = sorted(generator.choice(len(result), 2, replace=False))
                    if end_index > start_index + 1 and edge(result[start_index], result[end_index]):
                        result = result[:start_index + 1] + result[end_index:]
                assert all(edge(start, end) for start, end in zip(result, result[1:]))
                print('BUTTON_RRT_DENSE_PASS', side, iteration, len(result), flush=True)
                return result
    raise ValueError('Button empty RRT found no collision-free path within bounded search')
