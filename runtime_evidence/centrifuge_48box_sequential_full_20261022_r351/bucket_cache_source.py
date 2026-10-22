ORIGINAL_BUCKET_FUNCTION = """def bucket_relative_corners(poses, side):
    body = pose_matrix(poses['box48_' + side])
    bucket = pose_matrix(poses['centrifuge']) @ pose_matrix(poses['bucket_' + side])
    relative = np.linalg.inv(bucket) @ body
    corners = np.array(list(itertools.product((-1, 1), repeat=3))) * np.array([.1187450051085, .08814010043, .0595]) / 2
    return corners @ relative[:3, :3].T + relative[:3, 3]"""

CACHED_BUCKET_FUNCTION = """def bucket_relative_corners(poses, side):
    from cached_bucket import cached_bucket_corners
    return cached_bucket_corners(poses, side)"""
