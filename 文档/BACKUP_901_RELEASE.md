# 901 backup release contract

Release date: 2026-09-01 (Asia/Shanghai)

The canonical non-overwrite backup targets for the accepted under-three-minute
831-derived round-trip result are:

```text
/home/liu/h1_moveit/backups/901.tar.zst
/home/liu/h1_moveit/backups/901.tar.zst.sha256
```

The archive contains the complete current `h1_2_ros2_ws/` workspace under that
single relative top-level directory. The adjacent sidecar is the authoritative
machine-verifiable archive hash; the archive deliberately does not embed its
own digest.

The accepted Gazebo run is
`runtime_assets/candidate3_roundtrip_831_r15/`. Its strict timed metric from
`CANDIDATE3_ROUNDTRIP_831_FLOW_INVOCATION` through terminal
`CANDIDATE3_ROUNDTRIP_831_FLOW_COMPLETION=PASS` is `168.759167141 s`, below
the `180 s` limit. The motion window is `166.023644231 s`.

The sealed command authority is
`runtime_assets/candidate3_roundtrip_831_authority_v7/`:

```text
run id          6becfb17a6b1dcb61446c0d4a866ed701b69f61e14856a2501760468047baa27
manifest SHA    814a90cfca9e1c52c9ef2e536ac5b27f3bdb45bf21e318fcd3b807046fafc40d
commands SHA    f5d2f2aff2da41142b36dfc4acc7dd5acb5eb8129caeccb4203c78dce0c0053f
```

The run completed both cycles: right-hand-only door operation, cube placement
inside, relock, reopen, cube return to the sealed origin, and final relock.
The rosbag contains exactly 60 controller messages. Peak absolute torso yaw is
`44.25000000101588 deg`; terminal `left_handle_commands=0`; the generic handle
contact topic contains zero messages. The final latch is locked, all five
trajectory publisher counts are zero, and cube-origin translation error is
`0.8972242146953582 mm`.

Primary evidence inside the workspace:

```text
runtime_assets/candidate3_roundtrip_831_r15/RUN_SUMMARY.md
runtime_assets/candidate3_roundtrip_831_r15/candidate3_roundtrip_831_r15_demo.mp4
runtime_assets/candidate3_roundtrip_831_r15/candidate3_roundtrip_831_r15_under_3min.mp4
runtime_assets/candidate3_roundtrip_831_r15/final_state.png
runtime_assets/candidate3_roundtrip_831_r15/rosbag_831/
```

Both videos fully decode without ffmpeg errors. The related offline suite is
`166 passed`. Authority closure is `18/18` local and `7/7` external.

The archive and sidecar must be built as same-directory temporary regular
files, fully checked, and published with no-replace hard links. Final files
must be owned by `liu`, mode `0444`, and link count 1. Existing backups,
including `827.tar.zst`, `828.tar.zst`, and `831.tar.zst`, must remain
unchanged. The final 901 archive SHA-256 exists only in the adjacent sidecar
and the post-publication report to avoid a self-referential archive digest.
