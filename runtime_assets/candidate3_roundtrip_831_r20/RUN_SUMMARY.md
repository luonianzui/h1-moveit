# Candidate-3 831 v9/r17 requested-angle replay (r20)

Status: **PASS**

This is a fresh, non-overwriting visual replay of the proven v9/r17 flow.  It
uses the same trajectory, timing, controller authorization, and right-handle
executor semantics as v9/r17, with only a new authority/run identity and the
requested rear-right elevated Gazebo camera view.

The complete visible flow executed:

1. right hand opens the locked door;
2. left hand moves the cube from its origin into the incubator;
3. right hand closes and locks the door;
4. right hand opens the door again;
5. left hand returns the cube to its original table pose;
6. right hand closes and locks the door again.

The left hand never commands or contacts the door handle.

## Authority

- Semantic authority: `candidate3_roundtrip_831_authority_v9`
- Replay authority: `candidate3_roundtrip_831_authority_v12`
- Replay run: `candidate3_roundtrip_831_r20`
- Run id: `b142796b33b4d8ff38965bb561f1b7a618b32eca398e69dae77347c943aa2416`
- Replay manifest SHA-256: `99b31cf2f0400af1b8b1909596320eba3205c3526a31b3646dc44a0981271987`
- Replay commands SHA-256: `5a1412612356d1323b54d17b6c5384089dadadb6aa7c2eaf4101d6a2c1c30f9e`
- Right executor SHA-256: `7a1233641c26298a92b0e153328dfd76e5e561a2f5520fec40f1c11a165ecde8` (exact v9/r17 match)
- Controller authorization SHA-256: `6108f94f348e7d88ee96d94e587e7958bc52ec825fd1289803fb9b1bf8b5cb8c` (exact v9/r17 match)

## Runtime timing

Monotonic markers:

- flow invocation: `4463148299971 ns`
- motion window start: `4465553743752 ns`
- motion window end: `4628787055333 ns`
- terminal completion PASS: `4628787075393 ns`

Derived wall durations:

- invocation to motion start: `2.405443781 s`
- motion window: `163.233311581 s`
- invocation to terminal PASS: `165.638775422 s`
- requested video limit: `< 170 s`

Terminal result:

`CANDIDATE3_ROUNDTRIP_831=PASS cycles=2 cube=origin door=locked left_handle_commands=0 left_handle_contact_forbidden=true`

## Video

- File: `v9_r17_requested_angle_roundtrip_clean.mp4`
- Encoding: H.264, `1160x1518`, `10 fps`, `1662` frames
- Duration: `166.200000 s` (`3.800000 s` under the requested limit)
- Size: `5,573,086` bytes
- SHA-256: `6e4de2d11697b15b68b2749eeaee8979d7283a1c3f5d54ba73315b288828faaf`
- Camera position: `(-0.235, 0.5525, 1.4225)`
- Camera quaternion: `(0.034548296795, 0.086896763799, -0.367831264187, 0.925178646920)`
- Visual QA frames: `video_check_0.2s.png`, `video_check_42s.png`,
  `video_check_84s.png`, `video_check_126s.png`, `video_check_end.png`
- GUI entity selection was cleared before capture; no selection wireframe is
  present in the final video.

## Rosbag audit

Bag: `rosbag_831/rosbag_831_0.db3`

- total controller trajectory messages: `60`
- torso controller: `16`
- left arm controller: `16`
- right arm controller: `16`
- left gripper controller: `4`
- right gripper controller: `8`
- `/incubator/handle/contacts` messages: `0`

The recorder reported two transport-layer drops among high-rate messages.  All
60 required controller messages are present with exact topic counts.

## Final Gazebo state

- cube: origin pose
- latch: `locked`
- door support: detached
- cube support: detached
- final robot state: launch pose
- left-handle commands: `0`
- handle-contact messages: `0`
- final publishers on all five trajectory topics: `0`

Gazebo remains running at the successful final state and requested camera view.
