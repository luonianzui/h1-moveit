# Candidate-3 831 post-Mid360 full round-trip replay (v19/r29)

Status: **PASS**

This is a fresh, non-overwriting Gazebo replay of the proven v9/r17
round-trip action after the shared H1-2 description gained the Mid-360 sensor.
The complete natural-speed sequence executed visibly: the right hand operated
the door handle, the left hand placed the cube inside, the right hand closed
and locked the door, the right hand reopened it, the left hand returned the
cube to its table origin, and the right hand closed and locked the door again.

This run is a fixed-trajectory regression replay and visual artifact. It does
not replace the v9/r17 semantic authority and is not AprilTag dynamic
replanning evidence.

## Authority and identity

- Semantic authority: `candidate3_roundtrip_831_authority_v9`
- Replay authority: `candidate3_roundtrip_831_authority_v19`
- Replay run: `candidate3_roundtrip_831_r29`
- Run id: `dc2f34a6572037e52b3c770b878d15f07ad3e6843461577fd599e39fad375269`
- Builder SHA-256: `70be0e56c4f3dec395b1e5e4e720b023eabca9d0d46a67ba7621f0637314414e`
- Replay manifest SHA-256: `43c765450228557c1930787e25e9661eeed5fe34ffe2b80318ed3152aab8d7ff`
- Replay commands SHA-256: `752126eac221b9c5cea15fd433811f95992aba025278b381d93034ce59109612`
- Right executor SHA-256: `7a1233641c26298a92b0e153328dfd76e5e561a2f5520fec40f1c11a165ecde8`
- Controller authorization SHA-256: `6108f94f348e7d88ee96d94e587e7958bc52ec825fd1289803fb9b1bf8b5cb8c`
- Builder self-test: PASS, `writes_performed=false`
- Sealed authority audit: PASS, 18 files, 0 mismatches
- Identity-independent payload checks: `16/16 PASS`
- Action and timing payloads: unchanged from v9/r17 and v18/r28
- Preparation: PASS, controller commands issued: `0`

The new authority and run paths were absent before publication. The builder
published the authority once, and the run root was then created exclusively
with ordinary `mkdir`.

## Runtime timing

Monotonic markers:

- flow invocation: `85551926789090 ns`
- motion window start: `85554204227516 ns`
- motion window end: `85727086059212 ns`
- terminal completion PASS: `85727086074762 ns`

Derived wall durations:

- invocation to motion start: `2.277438426 s`
- motion window: `172.881831696 s`
- invocation to terminal PASS: `175.159285672 s`
- execution contract: `< 180 s` — PASS

Terminal result:

`CANDIDATE3_ROUNDTRIP_831=PASS cycles=2 cube=origin door=locked left_handle_commands=0 left_handle_contact_forbidden=true`

The first local wrapper attempt used the wrong JSON collection and exited on
an `AttributeError` before it reached `execv`; it issued no controller command.
The retry is explicitly marked in `flow_output.log` and executed the complete
`flow` argv directly from `commands_831.json` without modifying it.

## Video

File: `post_mid360_roundtrip_full_r29.mp4`

- Encoding: H.264 Main, `1160x1518`, `10 fps`, `yuv420p`
- Duration: `211.300000 s`
- Frames: `2113`
- Playback: original speed; continuous capture with no cuts or action reordering
- Size: `2,487,797` bytes
- SHA-256: `7f77658d9d7cd515503fe66e432b35892d0044ac80d37f6e3f0e9b4e85c1d6c7`
- Full decode: PASS
- Five-point visual QA: PASS in `video_contact_sheet.png`
- Capture region: Gazebo child window crop `1160x1518+0+0`
- Video recording started before the successful flow invocation and ended
  after terminal PASS.

Supporting images:

- `initial_camera_view.png`: `94637165882d84c1122c25bc02a2f759223b4f0082ab06cf1c056d568e22b399`
- `final_camera_view.png`: `2654007eca0f8b5f2af7da4cefb41a1bca59a252ae3035c4f5a197a74246fcfa`
- `video_contact_sheet.png`: `857c89ee45e9a20f61eaa16c759aa85c056b0234b04090233b2234eaaa4f1c8b`

## Camera pose

- position: `(0.612566351891, -0.757268905640, 1.447222232819)`
- quaternion `(x, y, z, w)`:
  `(0.150994822383, -0.101557202637, -0.815922021866, -0.548778593540)`
- The r28 user-selected pose was restored through `/gui/move_to/pose` and
  reverified both before recording and after the flow without drift.
- Pose record: `CAMERA_POSE_POST_MID360_R29.json`
- Pose record SHA-256: `0dc485be05c628d796dfedb6dc0b4ed12318a4c3a80ad3565bd2c7b233f30175`

## Rosbag audit

Bag: `rosbag_831/rosbag_831_0.db3`

- total controller trajectory messages: `60`
- torso controller: `16`
- left arm controller: `16`
- right arm controller: `16`
- left gripper controller: `4`
- right gripper controller: `8`
- `/incubator/handle/contacts` messages: `0`
- total bag messages: `207,554`
- bag duration: `425.634477434 s`
- all controller payload numbers finite: PASS
- all recorded joint telemetry numbers finite: PASS
- maximum commanded torso angle: `44.25 deg`
- maximum measured torso angle: `44.250000001018257 deg` — below `45 deg`
- SQLite integrity: PASS
- database SHA-256: `e3af46ad7c404f2f5a338b3e8c9cd2a4b5da071067a1da9aa63708bd68e6f9b2`
- metadata SHA-256: `fa120d5796b675eef626046deeb560833d3c4ad999cf0a39da1de57770210b0e`

Both the GUI video recorder and rosbag recorder received normal SIGINT after
flow PASS. MP4 decode, rosbag metadata, and SQLite integrity confirm flush
completion.

## Final Gazebo state

- cube pose: `(0.334108965184, 0.377058196732, 0.829999576072) m`
- cube RPY: `(-0.000000193231, 0.000000114583, 1.571452785417) rad`
- cube position error from spawn origin: `0.000123533196 m` — PASS
- door hinge: `-2.3251219838253058e-22 rad`
- handle latch joint: `-2.852694480443925e-16 rad`
- latch: `locked`
- door support: `detached`
- cube support: `detached`
- final robot state: launch pose
- left-handle commands: `0`
- handle-contact messages: `0`
- final publishers on all five trajectory topics: `0`
- support socket and support/flow/recording processes: absent after clean exit

Gazebo remains running at the successful final state and unchanged camera
view. At final audit the launch PID was `2436711`, Gazebo server PID `2436987`,
and Gazebo GUI PID `2436988`.

No backup was created because the user did not assign a new backup number.
All conclusions remain Gazebo-only; no physical robot connection or execution
was authorized.
