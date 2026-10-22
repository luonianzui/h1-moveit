# Candidate-3 831 combined Mid360-map round-trip replay (v21/r31)

Status: **PASS**

This is a fresh, non-overwriting Gazebo replay of the proven v9/r17
incubator round trip inside the completed Mid360 navigation map scene. The
scene contains the exact four-wall, 5 m square, 2.5 m high navigation
enclosure, the Mid360 ROS observation/localization pipeline, the H1-2 with
both grippers, the table, green cube, and incubator.

The complete fixed-trajectory sequence executed at natural speed: the right
hand opened the door, the left hand placed the cube inside, the right hand
closed and locked the door, the right hand reopened it, the left hand returned
the cube to its table origin, and the right hand closed and locked the door
again.

This run is a combined-scene fixed-trajectory regression and visual artifact.
It does not replace the v9/r17 semantic authority, does not prove AprilTag
dynamic replanning, and does not prove Nav2 locomotion while manipulating.

## Authority and identity

- Semantic authority: `candidate3_roundtrip_831_authority_v9`
- Combined-scene replay authority: `candidate3_roundtrip_831_authority_v21`
- Replay run: `candidate3_roundtrip_831_r31`
- Run id: `1072ae8ba1a4e59fe19caa14b958d5f1365e04e9fa77e7be0909acee5b33a54a`
- Builder SHA-256: `a316c07e7e248e676f46fd013e7821d1ee0c56290662a835e35dbab2dc22e6a2`
- Replay manifest SHA-256: `363314e291863c2e3ed8230e2a3549fcf6c17d73f913126f937f34e787666edb`
- Replay commands SHA-256: `dd06328648e7380798136097c758edb9cb52a1d3f4f8af1a5ddf2ebb81bc8fda`
- Combined world SHA-256: `6da0ddd8a9af867d427293b4bae1cc4a82e4c03ec9497755a9a835951b70e671`
- Combined launch SHA-256: `3b756c1d321e5dbd7e851031b4e1add33ee6dd4562cc2b2daee2f64598cd4492`
- Mid360 configuration SHA-256: `4786feb493efc8a9fee306ff90d247320b42ad44823826701d247a829f7cd27f`
- Incubator model SHA-256: `44ff23fc7077f0c232e2e69ff9d73aba336140d6a7951968c9c4241a97e73db9`
- Builder self-test: PASS, `writes_performed=false`
- Sealed authority audit: PASS, 19 files, 0 mismatches
- Non-scene action/timing/safety payloads: `14/14` byte-identical to v9/v20
- SDF validation and runtime preflight: PASS
- Preparation: exact PASS, controller commands issued: `0`

The authority and run paths were absent before publication. The builder
published the authority once; the run root was then created exclusively with
ordinary `mkdir`.

The immediately preceding v20/r30 identity used the old standalone action
world. When the user clarified that the replay must occur in the completed
radar map, v20/r30 was stopped before rosbag, support daemon, preparation, or
flow. It issued no controller command and is permanently marked
`ABORTED_PRE_MOTION.md`; it must not be reused.

## Combined scene and preparation

- Models present: `ground_plane`, `navigation_enclosure`, `h1_2`, `table`,
  `green_cube`, and `incubator`.
- Navigation enclosure: four walls, 5 m square, 2.5 m high.
- Action physics: 5 ms maximum step and real-time factor 1.0, unchanged from
  the proven action scene.
- Mid360 topics: `/livox/mid360/points`, `/scan`, `/map`,
  `/mid360_localization/pose`, and `/mid360_localization/status`.
- Fresh localization gate: valid, 720 points, four visible walls.
- Support daemon: one-shot bound-path claim PASS; socket mode 0600.
- Preparation marker:
  `CANDIDATE3_ROUNDTRIP_831_PREPARATION=PASS return_authority=valid preload=complete daemon=ready controller_commands=0`.

The first fresh-health checker required mathematically exact zero localization
and the first preparation checker expected one trajectory subscription even
though the active rosbag recorder correctly added a second subscription. Both
false negatives are preserved. Bounded retry audits accepted at most 2 cm /
0.01 rad localization and accounted for rosbag subscriptions; both retries
PASS without restarting or changing the scene.

Before the builder chain, two generated-install launch/world symlinks were
materialized as independent regular copies so their input identity was
auditable. The previous entries are preserved under
`runtime_evidence/candidate3_roundtrip_831_v20_build_20260914_r2/main_install_before/`;
`main_install_repair.txt` records PASS. No source model, trajectory, action
payload, v9/v19 authority, or successful historical run was modified.

## Runtime timing

Monotonic markers:

- flow invocation: `9777696637585 ns`
- motion window start: `9780403336902 ns`
- motion window end: `9944999045954 ns`
- terminal completion PASS: `9944999068274 ns`

Derived wall durations:

- invocation to motion start: `2.706699317 s`
- motion window: `164.595709052 s`
- invocation to terminal PASS: `167.302430689 s`
- execution contract: `< 180 s` — PASS

Terminal result:

`CANDIDATE3_ROUNDTRIP_831=PASS cycles=2 cube=origin door=locked left_handle_commands=0 left_handle_contact_forbidden=true`

## Mid360 localization at final state

- status: valid / reason `pass`
- pose `(x, y, yaw)`: `(0.000000, -0.005000, 0.000000)`
- point count: `720`
- visible walls: `4`
- wall point counts: `304/58/181/177`
- inlier fraction: `1.0`
- inlier RMS: `0.017826493411 m`
- robust RMS: `0.013824920886 m`

This is an observational localization check while the fixed manipulation
trajectory runs. The localizer does not command the robot in this replay.

## Video

File: `candidate3_roundtrip_831_r31_gui.mp4`

- Encoding: H.264 Main, `1160x1518`, `10 fps`, `yuv420p`
- Duration: `222.100000 s`
- Frames: `2221`
- Playback: original speed; continuous capture with no cuts or action
  reordering
- Size: `8,727,790` bytes
- SHA-256: `3c4dbf44dca9cba740c0998c9056c205691a5f7854d43bbc4f79a35b872be88a`
- Full decode: PASS
- Six-point visual QA: PASS in `video_contact_sheet.png`
- Capture: Gazebo child window `2000x1690`, cropped to the `1160x1518` world
  viewport
- The four-wall enclosure remains visible behind the robot and incubator.
- Recording began before flow invocation and ended after terminal PASS.

Supporting visual files:

- `view_before_flow.png`: wide full-map pre-flow view
- `view_action_before_flow.png`: restored action view before flow
- `video_initial_frame.png`: `7de55309823a57bb97e01678b431e8a4fbc289534481bcf41b73c0659d1ffb52`
- `video_final_frame.png`: `1363c298341c13b3ec8f15e792770dcd80ee0553ccbb645a950b6da8d96c19e1`
- `video_contact_sheet.png`: `142d5c5ba30b4e8aa7061d01009edf516ae983234c40e520408197203694dc37`

## Camera pose

- position: `(0.612566351891, -0.757268905640, 1.447222232819)`
- quaternion `(x, y, z, w)`:
  `(0.150994822383, -0.101557202637, -0.815922021866, -0.548778593540)`
- The trusted r29 action pose was restored through `/gui/move_to/pose` and
  reverified after flow with zero measured drift.
- Pose record: `CAMERA_POSE_COMBINED_MAP_R31.json`

## Rosbag audit

Bag: `rosbag_831/rosbag_831_0.db3`

- total controller trajectory messages: `60`
- torso controller: `16`
- left arm controller: `16`
- right arm controller: `16`
- left gripper controller: `4`
- right gripper controller: `8`
- `/incubator/handle/contacts` messages: `0`
- total bag messages: `287,055`
- bag duration: `583.744043463 s`
- selected numeric payloads checked: `6,328,174`
- all controller and joint telemetry numbers finite: PASS
- maximum commanded torso angle: `44.25 deg`
- maximum measured torso angle: `44.25000000101826 deg` — below `45 deg`
- SQLite integrity: `ok`
- database SHA-256: `5a87b340a802ba870ceeb8aa729f963a6ec6e96f83050c819cc25dec044bbdd4`
- metadata SHA-256: `147468aca3e2cdc0dfa4f950056fcfe5bae06b127fdd23ec4e3e74027f06236f`

The recorder reported two transport-layer losses among the high-rate
telemetry. The complete expected controller set is nevertheless present, all
60 payloads deserialize and are finite, handle contacts remain zero, and
SQLite integrity passes. The GUI video recorder and rosbag recorder each
received normal SIGINT after flow PASS and completed file flush.

## Final Gazebo state

- cube pose: `(0.334238508678, 0.377056800863, 0.829999476763) m`
- cube position error from spawn origin: `0.000245179529 m` — PASS
- door hinge: `-2.3251219838253058e-22 rad`
- handle latch joint: `-2.85269448041167e-16 rad`
- latch: `locked`
- door support: `detached`
- cube support: `detached`
- final robot state: exact model launch pose
- left-handle commands: `0`
- handle-contact messages: `0`
- final publishers on all five trajectory topics: `0`
- Mid360 localization: valid with all four walls visible
- camera pose: unchanged

The first final process audit matched its own shell command text and is
preserved as `final_audit_initial_process_selfmatch.json`. The PID-based retry
passes. Flow, support daemon, video recorder, rosbag recorder, support socket,
launch, Gazebo server/GUI, bridge, relay, and localizer are all absent after
normal shutdown; current live state is offline.

## Protection and backup boundary

Post-run hashes match the protected v9 manifest, r17 summary, v19 manifest,
r29 summary, incubator source model, and backup 914. In particular:

- v9 manifest: `b9e49b32c60979eb5a97d8ed3d5dc325a5d73e10b1985a45fd99c61f8848cd29`
- r17 summary: `859e626d9ad77fe9b164fc64856bb8f5ac6b7ab785e3ed1361114fea66561000`
- v19 manifest: `43c765450228557c1930787e25e9661eeed5fe34ffe2b80318ed3152aab8d7ff`
- r29 summary: `17f1f16b63e9727589d7e31d875b7474fd436a163af677bc25706dd0183c1c60`
- incubator source model: `44ff23fc7077f0c232e2e69ff9d73aba336140d6a7951968c9c4241a97e73db9`
- backup 914: `96ea710307e598a023503cc6361dd0795b924fb51e992dd3c51a1f6763f0ca48`

Backup 914 predates v20/r30, v21/r31, this report, and the accompanying handoff
updates. No backup was created in this run. All conclusions remain Gazebo-only;
no physical robot connection or execution was authorized.
