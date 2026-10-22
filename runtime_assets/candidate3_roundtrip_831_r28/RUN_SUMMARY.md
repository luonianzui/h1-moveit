# Candidate-3 831 v9/r17 user-selected-view replay (r28)

Status: **PASS**

This is a fresh, non-overwriting replay of the proven v9/r17 round-trip flow
from the second camera angle selected live by the user. The complete natural-
speed sequence executed visibly: right-hand door operation, left-hand cube
placement inside, right-hand close and lock, right-hand reopen, left-hand cube
return to its table origin, and right-hand final close and lock. The left hand
never commands or contacts the door handle.

## Authority

- Semantic authority: `candidate3_roundtrip_831_authority_v9`
- Replay authority: `candidate3_roundtrip_831_authority_v18`
- Replay run: `candidate3_roundtrip_831_r28`
- Run id: `5a8c0489413ecc6481507d88ed1c3ccf9a212fb3c4784ca30418170f4a8fab9c`
- Replay manifest SHA-256: `adbaf4a1f31df35031c966d58a036afc13dd1f8956dbb522f12d7ad0640c19e5`
- Replay commands SHA-256: `bfed84d1b8881c4d00db4893c323933d219628dfb64b9879925939b67406ad55`
- Right executor SHA-256: `7a1233641c26298a92b0e153328dfd76e5e561a2f5520fec40f1c11a165ecde8`
  (exact v9/r17 match)
- Controller authorization SHA-256: `6108f94f348e7d88ee96d94e587e7958bc52ec825fd1289803fb9b1bf8b5cb8c`
  (exact v9/r17 match)
- Identity-independent payload checks: `16/16 PASS`
- Timing contract: unchanged from v9/r17
- Preparation: `PASS`, controller commands issued: `0`

## Runtime timing

Monotonic markers:

- flow invocation: `5750815155090 ns`
- motion window start: `5752634548666 ns`
- motion window end: `5924396130722 ns`
- terminal completion PASS: `5924396150362 ns`

Derived wall durations:

- invocation to motion start: `1.819393576 s`
- motion window: `171.761582056 s`
- invocation to terminal PASS: `173.580995272 s`
- execution contract: `< 180 s` — PASS

Terminal result:

`CANDIDATE3_ROUNDTRIP_831=PASS cycles=2 cube=origin door=locked left_handle_commands=0 left_handle_contact_forbidden=true`

## Video

File: `v9_r17_user_requested_angle_roundtrip_full_r28.mp4`

- Encoding: H.264 Main, `1160x1518`, `10 fps`, `yuv420p`
- Duration: `189.500000 s`
- Frames: `1895`
- Playback: original speed; no cuts, retiming, or action reordering
- Size: `3,913,208` bytes
- SHA-256: `56548a1547a97e29fdd7f55f2aeacd56106db43b6489dbdfce755d45a00dce70`
- Full decode: PASS
- Five-point visual QA: PASS in `video_contact_sheet.png`
- Capture region: Gazebo client viewport `1160x1518+214+138`
- No mouse cursor or GUI entity-selection wireframe is present.

## User-set camera

- position: `(0.612566351891, -0.757268905640, 1.447222232819)`
- quaternion `(x, y, z, w)`:
  `(0.150994822383, -0.101557202637, -0.815922021866, -0.548778593540)`
- The pose was captured before restart, restored through `/gui/move_to/pose`,
  and reverified after the flow without drift.

## Rosbag audit

Bag: `rosbag_831/rosbag_831_0.db3`

- total controller trajectory messages: `60`
- torso controller: `16`
- left arm controller: `16`
- right arm controller: `16`
- left gripper controller: `4`
- right gripper controller: `8`
- `/incubator/handle/contacts` messages: `0`
- total bag messages: `151,001`
- bag duration: `312.801749143 s`

The recorder reported two transport-layer drops among high-rate telemetry.
All 60 required controller messages are present with exact topic counts.

## Final Gazebo state

- cube pose: `(0.334095, 0.377060, 0.829999) m`, RPY
  `(-0.000001, -0.000001, 1.571030) rad` — origin tolerance PASS
- door hinge: `-2.3251219838253058e-22 rad`
- handle latch joint: `-2.852694480390384e-16 rad`
- latch: `locked`
- door support: `detached`
- cube support: `detached`
- final robot state: launch pose
- left-handle commands: `0`
- handle-contact messages: `0`
- final publishers on all five trajectory topics: `0`

Gazebo remains running at the successful final state and exact user-selected
camera view.
