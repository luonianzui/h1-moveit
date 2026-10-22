# 915 backup release contract

Release date: 2026-09-15 (Asia/Shanghai)

The canonical non-overwrite backup targets are:

```text
/home/liu/h1_moveit/backups/915.tar.zst
/home/liu/h1_moveit/backups/915.tar.zst.sha256
```

The archive contains the complete current `h1_2_ros2_ws/` workspace under that
single safe relative top-level directory. It preserves symlinks as symlinks and
does not directly include the external `/home/liu/h1_moveit/backups/`
directory. The adjacent read-only sidecar is the authoritative
machine-verifiable archive digest. The archive deliberately excludes its own
digest and all post-publication evidence to avoid self-reference.

After successful publication, 915 becomes the latest protected full-workspace
restore and development template. "Template" means the immutable baseline to
restore or copy before later work; it does not make the archive itself writable
and does not replace any action, planning, or run authority.

The action semantic authority remains:

```text
authority  runtime_assets/candidate3_roundtrip_831_authority_v9/
run        runtime_assets/candidate3_roundtrip_831_r17/
manifest   b9e49b32c60979eb5a97d8ed3d5dc325a5d73e10b1985a45fd99c61f8848cd29
summary    859e626d9ad77fe9b164fc64856bb8f5ac6b7ab785e3ed1361114fea66561000
```

The latest complete Mid360 four-wall fixed-trajectory visual/regression
evidence is v23/r33. It preserves the v9/r17 non-scene action payload and shows
the two-cycle incubator task in Gazebo, but it is not AprilTag dynamic execution,
Nav2 walking, or navigation-after-vision evidence. Its key digests are:

```text
authority manifest  6bed442e7351876079796ca03d94f2657e7c4c086659fabf5e805691c1b53d68
run summary         3c15225512bd37afdb0c2f1c60c22ef70032caa50f227171e77d62319bbc0287
GUI MP4             503a05a2f09123ae77366f44f35bb1abc1d523a3cacfc6b18f47bb4943ea8ab3
```

This release also includes the independent 826 AprilTag A--L result, the D1/D1a
diagnostics, and the D1b fixed Tag-offset post-navigation right-hand-open
implementation and attempt11 plan-only matrix. D1b uses cabinet Tags 10/12 as
the rigid cabinet-pose authority, a fixed kinematic offset to derive the handle,
and Tag 11 only to cross-check the door. Its 24 successful cases and 13 expected
rejections do not constitute Gazebo execution or a guaranteed Nav2 residual
envelope; every admitted live pose still requires full IK and collision audit.
The complete two-cycle dynamic task still requires cube tags.

The archive includes the v22/r32 pre-motion fail-closed evidence, the v23/r33
successful replay and media, the post-914 handoff updates, the exact restored
support-plugin binary used by the sealed replay, and this pre-release contract.
It must not modify or overwrite v9, v19, v21, v23, any formal run, or backup 914.

The archive and sidecar must be created as new same-directory temporary regular
files, fully validated, and published with no-replace hard links. Final files
must be owned by `liu:liu`, mode `0444`, and have link count 1. Existing protected
backups 827, 828, 831, 901, 909, and 914 must pass their sidecars before and
after publication.

Required validation includes `zstd -t`, a complete tar member/type audit, one
safe relative top-level root, no absolute or parent-traversal members, no
duplicates, no hardlink/device/FIFO/socket members, no members below symlink
paths, critical inner-member hashes, full decoding of the archived latest r33
MP4, SQLite integrity of the archived r33 rosbag, sidecar `sha256sum -c`, and
final owner/mode/link-count checks. Any target collision or source mutation must
fail closed.

Restoring elsewhere may leave absolute symlink-install targets pointing at the
original workspace. Rebuild with the documented isolated ROS procedure rather
than assuming such links are portable.

Gazebo was brought to its locked, detached, zero-publisher terminal state and
then shut down normally before this backup cutoff. Backup creation authorizes no
simulation or navigation command, manipulation, reset, recovery, or physical
robot operation. All accepted robot conclusions remain Gazebo-only.
