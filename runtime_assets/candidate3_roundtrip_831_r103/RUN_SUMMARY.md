# Candidate3 roundtrip v24/r103

Date: 2026-09-16 CST

## Result

`CANDIDATE3_ROUNDTRIP_831=PASS cycles=2 cube=origin door=locked`

- Flow invocation to terminal PASS: `169.329060321 s` (`< 180 s` contract).
- Motion window: `166.788539065 s`.
- Cycle 1: right hand opens, left hand places the cube, right hand closes and locks.
- Cycle 2: right hand opens again, left hand returns the cube to its spawn origin, right hand closes and locks.
- Left-handle commands: 0; handle contact messages: 0.
- Controller messages: torso/right arm/left arm/left gripper/right gripper = `16/16/16/4/8` (60 total).
- Maximum commanded/measured torso: `44.25 / 44.25000000101488 deg`, both below 45 deg.
- Final cube position error from origin: `0.0005006679608470025 m`.
- Final door is q0, handle error is `5.332543533807687e-06 rad`, latch is locked, both supports are detached, robot is at launch pose, and all five trajectory publishers are zero.
- Final Mid360 result is valid with 720 points, all four walls visible, inlier/robust RMS `0.017840225306276748 / 0.013591709888403108 m`.

## Identity and provenance

- Builder: `build_candidate3_roundtrip_831_authority_v24.py`
- Builder SHA-256: `ff4ea7b3dbb701105467c43da270821ce3015a9e4be7838a1f786238891a3cc6`
- Authority: `runtime_assets/candidate3_roundtrip_831_authority_v24/`
- Run id: `7ec6d3ba4ce65b1beb14d7e0dde24aa3d3df0cd3d44a1895da442fbb7a7335c8`
- Manifest SHA-256: `8710dcbe597c78b96b0afa658ecbf30730feeb1188c2ca2b7df067f31e8617cf`
- Commands SHA-256: `de524b0b0c70a6c5938b65bb787152723c6acc14760dd84d7f6dd79a894cae0e`
- All 14 non-scene action/timing/safety payloads are byte-identical to v9.
- Launch/world/Mid360 scene payloads are byte-identical to v21.
- The contract-bound support plugin SHA-256 is `eb85e8b833a6cbaa0fa26eab09e6a983ba2e36243bf37a12ebb9001c19e95d8b`.
- v9/r17, v21/r31, v23/r33, incubator model, and backup 915 protected hashes matched before and after the run.

## Evidence

- Corrected final audit: `final_audit_retry1.json` (`PASS`).
- Rosbag audit: `rosbag_audit.json` (`PASS`, SQLite integrity `ok`, 189469 messages).
- GUI video: `candidate3_roundtrip_831_r103_gui.mp4`, SHA-256
  `a5944e34a5db992faa0d628f1e5d7e398ce38dfbf14ff4e07bd5ec49c39de8c8`,
  204.5 s, 2045 frames, 1160 x 1518 at 10 fps, full decode PASS.
- Initial/final frame viewpoint audit PASS: mean absolute pixel difference `0.2962603925310072`, with only `0.0021171232565535413` of pixels above channel-MAE 5.
- Gazebo GUI/server remain open in the safe terminal state for user inspection; flow, support daemon, video recorder, and rosbag recorder have exited, the support socket is absent, and no trajectory publisher remains.

This is a fixed-trajectory v9/r17 semantic replay in the v21 four-wall Mid360
scene. It is visual and regression evidence only. It does not execute or prove the
D1b post-navigation AprilTag fixed-offset replanner, Nav2 legged walking, or a full
dynamic-tag double cycle. No real robot was connected or authorized. Backup 915
was not modified and no new backup was created.
