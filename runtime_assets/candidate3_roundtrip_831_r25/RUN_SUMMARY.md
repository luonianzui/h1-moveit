# Candidate-3 831 v9/r17 front-view replay (r25)

Status: **PASS**

This is a fresh, non-overwriting replay of the proven v9/r17 flow from the
front camera angle selected by the user.  All 16 identity-independent
execution, trajectory, world, controller, and safety payloads were checked
byte-for-byte before launch.  The complete two-cycle flow executed visibly:

1. right hand opens the locked door;
2. left hand moves the cube from its origin into the incubator;
3. right hand closes and locks the door;
4. right hand opens the door again;
5. left hand returns the cube to its original table pose;
6. right hand closes and locks the door again.

The left hand never commands or contacts the door handle.

## Authority

- Semantic authority: `candidate3_roundtrip_831_authority_v9`
- Replay authority: `candidate3_roundtrip_831_authority_v17`
- Replay run: `candidate3_roundtrip_831_r25`
- Run id: `37119b31e6481cd68927f7cfeb420f5a831ee1929f082d2271a3631ab847db14`
- Replay manifest SHA-256: `91df60408ca4dc6408316d49ff60011d8de8a37b6512e5319e4683b564694d2e`
- Replay commands SHA-256: `2afde9e97863a2662b9ddfbb269af297d55f3bfb5cb7ddca75d4e304dbbabed4`
- Right executor SHA-256: `7a1233641c26298a92b0e153328dfd76e5e561a2f5520fec40f1c11a165ecde8` (exact v9/r17 match)
- Controller authorization SHA-256: `6108f94f348e7d88ee96d94e587e7958bc52ec825fd1289803fb9b1bf8b5cb8c` (exact v9/r17 match)
- Identity-independent payload checks: `16/16 PASS`
- Timing contract: unchanged from v9/r17

## Runtime timing

Monotonic markers:

- flow invocation: `3959950573193 ns`
- motion window start: `3962016479255 ns`
- motion window end: `4136824068611 ns`
- terminal completion PASS: `4136824092631 ns`

Derived wall durations:

- invocation to motion start: `2.065906062 s`
- motion window: `174.807589356 s`
- invocation to terminal PASS: `176.873519438 s`
- execution contract: `< 180 s` — PASS

Terminal result:

`CANDIDATE3_ROUNDTRIP_831=PASS cycles=2 cube=origin door=locked left_handle_commands=0 left_handle_contact_forbidden=true`

## Video

File: `v9_r17_front_view_roundtrip_full_r25.mp4`

- Encoding: H.264, `1160x1518`, `10 fps`
- Duration: `177.600000 s`
- Frames: `1776`
- Playback: original speed; no cuts, retiming, or action reordering
- Size: `3,252,565` bytes
- SHA-256: `7461aef7b6fc0e24699175473e5ad25713a3e158c869f603bb603407cfc938ea`
- Full decode: PASS
- Visual QA: start, three intermediate states, and final frame PASS in
  `video_contact_sheet.png`
- No mouse cursor or GUI entity-selection wireframe is present.

## User-set camera

- position: `(0.930465102196, 0.897991418839, 1.554799318314)`
- restored quaternion:
  `(0.227072283626, 0.098219178617, -0.889286220074, 0.384657144547)`
- the exact requested pose was captured before restart, restored in the fresh
  world, and reverified after PASS

## Rosbag audit

Bag: `rosbag_831/rosbag_831_0.db3`

- total controller trajectory messages: `60`
- torso controller: `16`
- left arm controller: `16`
- right arm controller: `16`
- left gripper controller: `4`
- right gripper controller: `8`
- `/incubator/handle/contacts` messages: `0`
- total bag messages: `147,856`
- bag duration: `306.775954206 s`

The recorder reported three transport-layer drops among high-rate messages.
All 60 required controller messages are present with exact topic counts.

## Final Gazebo state

- cube pose: `(0.333942, 0.377040, 0.829999) m`, RPY
  `(-0.000002, -0.000001, 1.570830) rad` — origin tolerance PASS
- door hinge: `-2.32512198383e-22 rad`
- handle latch joint: `-2.85269448029e-16 rad`
- latch: `locked`
- door support: `detached`
- cube support: `detached`
- final robot state: launch pose
- left-handle commands: `0`
- handle-contact messages: `0`
- final publishers on all five trajectory topics: `0`

Gazebo remains running at the successful final state and exact user-selected
front view.
