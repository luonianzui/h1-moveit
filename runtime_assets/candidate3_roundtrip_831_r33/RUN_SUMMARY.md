# Candidate3 roundtrip v23/r33

Date: 2026-09-15 CST

## Result

`CANDIDATE3_ROUNDTRIP_831=PASS cycles=2 cube=origin door=locked`

- Flow invocation to terminal PASS: `176.938775168 s` (`< 180 s` contract).
- Motion window: `174.417793405 s`.
- Cycle 1: right hand opens, left hand places the cube, right hand closes and locks.
- Cycle 2: right hand opens again, left hand returns the cube to its spawn origin, right hand closes and locks.
- Left-handle commands: 0; handle contact messages: 0.
- Controller messages: torso/right arm/left arm/left gripper/right gripper = `16/16/16/4/8` (60 total).
- Maximum commanded/measured torso: `44.25 / 44.25000000101826 deg`, both below 45 deg.
- Final cube position error from origin: `0.0005300700159671292 m`.
- Final door and handle joints are q0, latch is locked, both supports are detached, robot is at launch pose, and all five trajectory publishers are zero.
- Final Mid360 result is valid with 720 points, all four walls visible, inlier/robust RMS `0.01801446925910032 / 0.014279130273725025 m`.

## Identity and provenance

- Builder: `build_candidate3_roundtrip_831_authority_v23.py`
- Builder SHA-256: `f8678a1ca3124170347aabe8312ea3ffe356fd70e4264ba7ed7edf637e5c887a`
- Authority: `runtime_assets/candidate3_roundtrip_831_authority_v23/`
- Run id: `edb3a20aaadcc3823add641210bed5d1d62b7a6fbe1dc549d3613f455788648d`
- Manifest SHA-256: `6bed442e7351876079796ca03d94f2657e7c4c086659fabf5e805691c1b53d68`
- Commands SHA-256: `e7aaabf28a523e1b39e8c4f8bd5ee8fa3f5018624b5105ae32cee114a2cad29e`
- All 14 non-scene action/timing/safety payloads are byte-identical to v9.
- Launch/world/Mid360 scene payloads are byte-identical to v21.
- The contract-bound support plugin was restored from protected backup 914 before this fresh identity was launched; its SHA-256 is `eb85e8b833a6cbaa0fa26eab09e6a983ba2e36243bf37a12ebb9001c19e95d8b`.

The immediately preceding v22/r32 identity failed closed at the pre-action support
plugin hash gate, issued zero controller commands, and was not reused.

## Evidence

- Corrected final audit: `final_audit_retry1.json` (`PASS`).
- Rosbag audit: `rosbag_audit.json` (`PASS`, SQLite integrity `ok`).
- GUI video: `candidate3_roundtrip_831_r33_gui.mp4`, SHA-256
  `503a05a2f09123ae77366f44f35bb1abc1d523a3cacfc6b18f47bb4943ea8ab3`,
  192.4 s, 1924 frames, 1160 x 1518 at 10 fps, full decode PASS.
- Camera pose remained equal to the protected r31 action view.

This is a fixed-trajectory v9/r17 semantic replay in the v21 four-wall Mid360
scene. It is visual and regression evidence only. It does not execute or prove the
D1b post-navigation AprilTag fixed-offset replanner, Nav2 legged walking, or a full
dynamic-tag double cycle. No real robot was connected or authorized. Backup 914
was not modified and no new backup was created.
