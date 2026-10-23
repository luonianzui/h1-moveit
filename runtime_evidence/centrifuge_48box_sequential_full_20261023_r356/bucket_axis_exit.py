import copy
import itertools
import math

import numpy as np


def bucket_axis_exit_targets(request, first, last, count, rims, pose_matrix, interpolate):
    if (request.get('mode') != 'retrieval_transport' or request.get('retrieval') is not True
            or count != 81 or set(first) != {'left','right'} or set(last) != set(first)
            or set(request.get('original_runtime_holder_references',{})) != set(first)
            or set(request.get('assignment',{})) != set(first)
            or set(request['assignment'].values()) != {'left','right'}):
        raise ValueError('Only original actual two-holder retrieval lift may receive basket-axis exit geometry')
    machine = pose_matrix(request['poses']['centrifuge'])
    exits = copy.deepcopy(first)
    size = np.array([.1187450051085,.08814010043,.0595])
    vertices = np.array(list(itertools.product((-1.,1.),repeat=3))) * size / 2
    for hand,payload in request['assignment'].items():
        bucket = machine @ pose_matrix(request['poses']['bucket_'+payload])
        body = np.asarray(first[hand])
        if body.shape != (4,4) or not np.isfinite(body).all() or not math.isfinite(rims[payload]):
            raise ValueError('Finite actual rigid body and original basket collision rim required')
        local = np.linalg.inv(bucket) @ body
        corners = vertices @ local[:3,:3].T + local[:3,3]
        distance = max(0.,float(rims[payload] + .006 - corners[:,2].min()))
        exits[hand][:3,3] = (body[:3,3] - bucket[:3,0]*local[0,3]
                            - bucket[:3,1]*local[1,3] + bucket[:3,2]*distance)
        relative = np.linalg.inv(bucket) @ exits[hand]
        bottom = (vertices @ relative[:3,:3].T + relative[:3,3])[:,2].min()
        if abs(relative[0,3]) > 1e-12 or abs(relative[1,3]) > 1e-12 or bottom < rims[payload]+.006-1e-12:
            raise ValueError('Centered rigid exit must clear original collision rim by six millimeters')
    targets = list(interpolate(first,exits,41)) + list(interpolate(exits,last,41))[1:]
    assert len(targets) == count
    for hand in first:
        assert np.allclose(targets[0][hand],first[hand],atol=1e-12,rtol=0)
        assert np.allclose(targets[-1][hand],last[hand],atol=1e-12,rtol=0)
    return targets
