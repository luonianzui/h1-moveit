# Candidate-3 831 v9/r17 user-set-angle replay (r24)

Status: **PASS**

This is a fresh, non-overwriting replay of the proven v9/r17 flow from the
Gazebo camera angle set by the user.  All 16 identity-independent execution,
trajectory, world, controller, and safety payloads were checked byte-for-byte
before launch.  The complete two-cycle flow executed visibly:

1. right hand opens the locked door;
2. left hand moves the cube from its origin into the incubator;
3. right hand closes and locks the door;
4. right hand opens the door again;
5. left hand returns the cube to its original table pose;
6. right hand closes and locks the door again.

The left hand never commands or contacts the door handle.

## Authority

- Semantic authority: `candidate3_roundtrip_831_authority_v9`
- Replay authority: `candidate3_roundtrip_831_authority_v16`
- Replay run: `candidate3_roundtrip_831_r24`
- Run id: `b1523d7a83b6ca7be179519c336393c4666d2202804b919462bb11b3e4398075`
- Replay manifest SHA-256: `475f61aba9eb91d62679b7b8241e476cdf80de88a8ecd8d06d5f6e30bbb6c3c6`
- Replay commands SHA-256: `0b4da78c4e41f7941850657bf215f95a0597b64aeb0fbb16e4216cf9f5ecfbea`
- Right executor SHA-256: `7a1233641c26298a92b0e153328dfd76e5e561a2f5520fec40f1c11a165ecde8` (exact v9/r17 match)
- Controller authorization SHA-256: `6108f94f348e7d88ee96d94e587e7958bc52ec825fd1289803fb9b1bf8b5cb8c` (exact v9/r17 match)
- Identity-independent payload checks: `16/16 PASS`
- Timing contract: byte-for-byte semantic continuation from v9/r17

## Runtime timing

Monotonic markers:

- flow invocation: `2454409428954 ns`
- motion window start: `2457303090622 ns`
- motion window end: `2627725857366 ns`
- terminal completion PASS: `2627725875336 ns`

Derived wall durations:

- invocation to motion start: `2.893661668 s`
- motion window: `170.422766744 s`
- invocation to terminal PASS: `173.316446382 s`
- execution contract: `< 180 s` — PASS

Terminal result:

`CANDIDATE3_ROUNDTRIP_831=PASS cycles=2 cube=origin door=locked left_handle_commands=0 left_handle_contact_forbidden=true`

## Video

Preferred natural-speed delivery file:
`v9_r17_user_set_angle_roundtrip_full_r24.mp4`

- Encoding: H.264, `1160x1518`, `10 fps`
- Duration: `174.000000 s`
- Frames: `1740`
- Playback: original speed; no segment cuts, retiming, or action reordering
- Size: `3,992,503` bytes
- SHA-256: `f90fe3b97b66e9e022f73ea64e463b2398aff63644e66a44db5550e7f0c21bbd`
- Full decode: PASS
- Visual QA: start, three intermediate states, and final frame PASS in
  `video_contact_sheet_clean.png`
- No mouse cursor or GUI entity-selection wireframe is present.

The optional under-170-second version is retained as
`v9_r17_user_set_angle_roundtrip_under170_clean_r24.mp4`: H.264,
`1160x1518`, `10.3 fps`, `1740/1740` frames, `168.932039 s`, uniform `1.03x`
playback, SHA-256
`1eb56c27ad0761a59275e05d5190fa9c1d81016cc5be0d97e73c1297df619f0f`.

## User-set camera

- position: `(-0.017454603687, 0.927346527576, 1.722714662552)`
- quaternion: `(0.195518195629, 0.289005309343, -0.525120139122, 0.776207089424)`
- the exact pose was restored after the fresh restart and reverified after PASS

## Rosbag audit

Bag: `rosbag_831/rosbag_831_0.db3`

- total controller trajectory messages: `60`
- torso controller: `16`
- left arm controller: `16`
- right arm controller: `16`
- left gripper controller: `4`
- right gripper controller: `8`
- `/incubator/handle/contacts` messages: `0`
- total bag messages: `149,294`
- bag duration: `310.438798967 s`

The recorder reported two transport-layer drops among high-rate messages.  All
60 required controller messages are present with exact topic counts.

## Final Gazebo state

- cube pose: `(0.333759, 0.377057, 0.829999) m`, RPY
  `(-0.000002, 0.000000, 1.571280) rad` — origin tolerance PASS
- door hinge: `-4.90915149938e-22 rad`
- handle latch joint: `5.64555850858e-06 rad`
- latch: `locked`
- door support: `detached`
- cube support: `detached`
- final robot state: launch pose
- left-handle commands: `0`
- handle-contact messages: `0`
- final publishers on all five trajectory topics: `0`

Gazebo remains running at the successful final state and exact user-set camera
view.
