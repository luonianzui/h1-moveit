# 909 backup release contract

Release date: 2026-09-09 (Asia/Shanghai)

The canonical non-overwrite backup targets are:

```text
/home/liu/h1_moveit/backups/909.tar.zst
/home/liu/h1_moveit/backups/909.tar.zst.sha256
```

The archive contains the complete current `h1_2_ros2_ws/` workspace under
that single safe relative top-level directory. It preserves symlinks as
symlinks and does not directly include the external `backups/` directory. The
adjacent sidecar is the authoritative machine-verifiable archive hash; the
archive deliberately does not embed its own digest.

The latest complete Gazebo replay included in this release is:

```text
authority   runtime_assets/candidate3_roundtrip_831_authority_v19/
run         runtime_assets/candidate3_roundtrip_831_r29/
run id      dc2f34a6572037e52b3c770b878d15f07ad3e6843461577fd599e39fad375269
result      PASS / cycles=2 / cube=origin / door=locked
timing      invocation-to-PASS 175.159285672 s
manifest    43c765450228557c1930787e25e9661eeed5fe34ffe2b80318ed3152aab8d7ff
commands    752126eac221b9c5cea15fd433811f95992aba025278b381d93034ce59109612
video       7f77658d9d7cd515503fe66e432b35892d0044ac80d37f6e3f0e9b4e85c1d6c7
rosbag db3  e3af46ad7c404f2f5a338b3e8c9cd2a4b5da071067a1da9aa63708bd68e6f9b2
```

The rosbag contains exactly 60 controller messages distributed as
`16/16/16/4/8`; handle-contact messages and left-handle commands are zero;
measured peak torso angle is below 45 degrees. Final cube origin, q0 door and
handle, locked latch, detached supports, launch robot pose, and zero final
trajectory publishers all pass. The MP4 contains 2113 H.264 Main frames at
1160x1518 and 10 fps and fully decodes.

The action semantic authority remains
`runtime_assets/candidate3_roundtrip_831_authority_v9/` with accepted run
`runtime_assets/candidate3_roundtrip_831_r17/`. v19/r29 is a fixed-trajectory
post-Mid360 regression replay and does not replace the v9/r17 semantics or the
independent 826 AprilTag dynamic replanning evidence.

This release also includes the Mid-360 model, 2.5 m navigation enclosure,
complete 360-degree scan evidence, Nav2 AMCL fixed-localization and
planner-only results, and the fail-closed V12 RL locomotion interface contract.
It does not claim moving navigation, Gazebo RL policy deployment, or physical
robot execution.

The archive and sidecar must be created as same-directory temporary regular
files, fully validated, and published with no-replace hard links. Final files
must be owned by `liu`, mode `0444`, and link count 1. Existing protected
backups, including `827.tar.zst`, `828.tar.zst`, `831.tar.zst`, and
`901.tar.zst`, must remain unchanged.

Required validation includes `zstd -t`, complete `tar --zstd -tf`, one safe
relative top-level root, no absolute or parent-traversal members, no duplicate
members, no hardlink/device/FIFO members, no members below symlink paths,
presence and inner hashes of the v19/r29 authority and core evidence, adjacent
sidecar `sha256sum -c`, and final owner/mode/link-count checks.

Restoring the archive elsewhere may leave absolute symlink-install targets
pointing at the original workspace. Rebuild with the documented isolated
ROS/Miniconda procedure instead of assuming such links are portable.

All accepted conclusions are Gazebo-only. No physical robot connection or
execution is authorized by this backup.
