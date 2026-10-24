ORIGINAL_POSE_FUNCTION = """def pose_matrix(value):
    matrix = np.eye(4)
    matrix[:3, 3] = [value.get('position', {}).get(axis, 0) for axis in 'xyz']
    orientation = value.get('orientation', {})
    matrix[:3, :3] = Rotation.from_quat([orientation.get(axis, 0) for axis in 'xyzw']).as_matrix()
    if not np.isfinite(matrix).all():
        raise ValueError('Non-finite body pose')
    return matrix"""

CACHED_POSE_FUNCTION = """def pose_matrix(value):
    from cached_pose import cached_pose_matrix
    return cached_pose_matrix(value)"""
