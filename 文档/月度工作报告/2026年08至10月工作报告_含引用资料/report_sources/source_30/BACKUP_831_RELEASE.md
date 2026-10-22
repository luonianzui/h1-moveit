# 831 backup release contract

Release date: 2026-08-31 (Asia/Shanghai)

The canonical non-overwrite backup targets for the accepted slow certified
baseline are:

```text
/home/liu/h1_moveit/backups/831.tar.zst
/home/liu/h1_moveit/backups/831.tar.zst.sha256
```

The archive contains the complete current `h1_2_ros2_ws/` workspace under that
single relative top-level directory. The adjacent sidecar is the authoritative
machine-verifiable archive hash; the archive does not embed its own digest.

The accepted full-flow run is
`runtime_assets/candidate3_live_829_final_r17/`. Its strict certificate is
`candidate3_live_certificate_829.json`, SHA-256
`6a36629a2834c6ea2673609a8034a826864bdc6475bfb9d1f192623777ba1801`.
The run ID is
`c29cd318c5a6b462dc0aa928d51cfcc77f2d6f3dcf278d24c84e4ed330cc456d`.

The sealed command authority is
`runtime_assets/candidate3_live_829_authority_v18/`, whose manifest SHA-256 is
`0679eb7e9169db5fdc309284eedf8dffd04f79fd1cff0e977306f1024f1ae320`.
The exact FCL authority is
`runtime_assets/right_handle_8263_natural/variable_torso_v4_829/live_fcl_authority_v6_31j/`,
whose manifest SHA-256 is
`c267d29253e0b68e14fd085f5ba4e024104664a5fe699ea76377dd976160bfbb`.

The certified baseline has table x = 0.600 m and incubator x = 0.590 m. The
right hand alone operates the handle; left-handle contacts are zero and the
minimum certified left-handle clearance is 0.05 m. Peak absolute torso yaw is
44.25 degrees, below the 45-degree user limit. The full video, rosbag, exact
FCL replay, seam checks, terminal pose, latch state, and certificate all pass.

The archive and sidecar must be validated from same-directory temporary
regular files and published with no-replace hard links. Final files must be
owned by `liu`, mode `0444`, and link count 1. Existing backups, including
`826.3.tar.zst`, `827.tar.zst`, and `828.tar.zst`, must remain unchanged.

This 831 release deliberately precedes the requested 90-second optimization.
It preserves the accepted slow-mode reference; it does not claim that the
full flow completes within 90 seconds or that slow mode has been removed.
