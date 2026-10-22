# 914 backup release contract

Release date: 2026-09-14 (Asia/Shanghai)

The canonical non-overwrite backup targets are:

```text
/home/liu/h1_moveit/backups/914.tar.zst
/home/liu/h1_moveit/backups/914.tar.zst.sha256
```

The archive contains the complete current `h1_2_ros2_ws/` workspace under that
single safe relative top-level directory. It preserves symlinks as symlinks and
does not directly include the external `/home/liu/h1_moveit/backups/`
directory. The adjacent read-only sidecar is the authoritative
machine-verifiable archive digest; the archive deliberately does not embed its
own digest.

This release includes all protected incubator action assets and evidence,
including the v9/r17 semantic authority, v19/r29 fixed-trajectory visual replay,
and the independent 826 AprilTag A--L evidence. It also includes the Mid360 and
Nav2 integration, the official Unitree H1-2 Gazebo gait policy, the r17 accepted
loose navigation baseline, the r21 fail-closed 30-second supervisor, all
rejected optimization identities through r27, and the r28 full-scene user-view
demonstration and its machine-readable audit.

The action semantic authority remains:

```text
authority  runtime_assets/candidate3_roundtrip_831_authority_v9/
run        runtime_assets/candidate3_roundtrip_831_r17/
manifest   b9e49b32c60979eb5a97d8ed3d5dc325a5d73e10b1985a45fd99c61f8848cd29
summary    859e626d9ad77fe9b164fc64856bb8f5ac6b7ab785e3ed1361114fea66561000
```

The latest incubator visual replay remains v19/r29, whose manifest and summary
digests are `43c765450228557c1930787e25e9661eeed5fe34ffe2b80318ed3152aab8d7ff`
and `17f1f16b63e9727589d7e31d875b7474fd436a163af677bc25706dd0183c1c60`.
It does not replace the v9/r17 semantics or the independent AprilTag evidence.

The latest navigation display evidence is:

```text
run       runtime_evidence/unitree_h12_incubator_nav2_user_view_20260914_r28/
result    Nav2 goal succeeded; strict stable hold rejected
terminal  physical/localized error 0.070414705/0.055000000 m
peak      post-Nav2 physical error 0.126195152 m
REPORT    b804635d52ee5c4a81263962d8e2c16cfb1ac0e2366d96e8907d365fe962c47a
audit     796265b485e5b146d514e2eb1b999fb90962c5c0de7efeb6f9ffbf8d89ca185a
```

r28 is a GUI demonstration of successful localized arrival, not a strict
30-second acceptance PASS. The current production source remains the unchanged
r17 gait plus r21 supervisor. The next authorized-zero-command slew concept is
only a future hypothesis and is not claimed as implemented by this backup.

The archive and sidecar must be created as same-directory temporary regular
files, fully validated, and published with no-replace hard links. Final files
must be owned by `liu`, mode `0444`, and have link count 1. Existing protected
backups, including 827, 828, 831, 901, and 909, must remain unchanged.

Required validation includes `zstd -t`, a complete tar member and type audit,
one safe relative top-level root, no absolute or parent-traversal members, no
duplicate members, no hardlink/device/FIFO/socket members, no members below
symlink paths, critical inner-member hash comparison, adjacent sidecar
`sha256sum -c`, and final owner/mode/link-count checks.

Restoring the archive elsewhere may leave absolute symlink-install targets
pointing at the original workspace. Rebuild with the documented isolated ROS
procedure instead of assuming such links are portable.

All accepted conclusions are Gazebo-only. This release grants no physical robot
connection or execution permission and does not authorize manipulation or
navigation commands during backup creation.
