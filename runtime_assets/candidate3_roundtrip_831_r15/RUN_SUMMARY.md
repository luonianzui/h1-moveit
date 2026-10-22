# Candidate 3 roundtrip 831 r15 — under-three-minute verified run

Date: 2026-09-01 (Asia/Shanghai)

## Outcome

`PASS`

The live Gazebo run completed the sealed two-cycle sequence:

1. Right hand opens the door.
2. Left hand places the cube inside.
3. Right hand closes and locks the door.
4. Right hand opens the door again.
5. Left hand retrieves the cube to its sealed origin pose.
6. Right hand closes and locks the door again.

Terminal verdict:

```text
CANDIDATE3_ROUNDTRIP_831=PASS cycles=2 cube=origin door=locked left_handle_commands=0 left_handle_contact_forbidden=true
```

## Strict under-three-minute timing

The contracted metric is the timed flow invocation through the terminal PASS,
not merely the robot motion window.

- flow invocation monotonic ns: `13656610030522`
- motion start monotonic ns: `13659345537282`
- motion end monotonic ns: `13825369181513`
- terminal PASS monotonic ns: `13825369197663`
- invocation to motion start: `2.735506760 s`
- motion window: `166.023644231 s`
- invocation to terminal PASS: `168.759167141 s`
- requirement: `< 180.000000000 s` — PASS
- remaining margin: `11.240832859 s`

The one-time, non-motion preparation ran before the timed invocation.  Its
exact PASS certified the return authority, preload, daemon READY state, and
zero controller commands.  The timed flow then repeated the return-authority
dry validation and completed a one-shot run-id/nonce/SO_PEERCRED/bound-path
socket claim before the first motion.

No trajectory, time scale, velocity setting, settle gate, or controller
message was shortened.  The right-handle executor, middle executor, and return
plan remain byte-identical to the preceding sealed authority.

## Motion and safety evidence

The rosbag contains exactly 60 controller trajectory messages:

- torso: `16`
- left arm: `16`
- right arm: `16`
- left gripper: `4`
- right gripper: `8`

Maximum measured absolute torso angle from `/joint_states`:

- `0.7723081940252212 rad`
- `44.25000000101588 deg`
- user limit: `45 deg` — PASS

Handle evidence:

- terminal validator: `left_handle_commands=0`
- `/incubator/handle/contacts`: `0` recorded messages
- the sealed flow/controller contract forbids left-hand handle contact

## Verified final live state

- door position: `-2.3251219838253058e-22 rad`
- handle position: `-2.8526944803797839e-16 rad`
- latch: `locked`
- cube world position: `(0.33310496673346812, 0.37706266180715592, 0.82999950891864349) m`
- sealed cube origin: `(0.334, 0.377, 0.830) m`
- cube origin translation error: `0.8972242146953582 mm`
- cube origin orientation error: `0.01956063078223299 deg`
- active trajectory publishers after completion: `0` on all five controller topics
- Gazebo GUI remains running at the successful final state

## Recording

Trimmed demonstration:

- file: `candidate3_roundtrip_831_r15_demo.mp4`
- H.264 / yuv420p / `1160x1000` / constant `8 fps`
- duration: `176.000000 s`
- frames: `1408`
- SHA-256: `4edc7222b01e81253e8050094f4435c236b2fa91352d909421fc226a3938e01d`
- full decode: PASS, zero ffmpeg errors

Untrimmed audit recording:

- file: `candidate3_roundtrip_831_r15_under_3min.mp4`
- H.264 / yuv420p / `1160x1000` / constant `8 fps`
- duration: `307.375000 s`
- frames: `2459`
- SHA-256: `70105eba766d5248c3babe9c1ed8199cde34fc35f8806d6f2b4913654df1327c`
- full decode: PASS, zero ffmpeg errors

Rosbag:

- path: `rosbag_831/`
- duration: `326.844534678 s`
- total messages: `159656`
- all 60 controller trajectory messages are present
- recorder reported two transport-layer samples lost on shutdown; the exact
  controller counts, terminal validators, and final live-state checks are complete

## Admission correction and tests

The preceding r14 attempt stopped before motion with zero controller commands.
Its claim compared the filesystem inode created by `bind()` with the unrelated
kernel socket inode returned by `fstat(server.fileno())`.  V7 now pins and
reports the stable bound pathname identity from READY through claim, while
still rejecting pathname replacement.

- related offline suite: `166 passed`
- explicit valid-connect bound-path identity test: PASS
- explicit pathname replacement rejection test: PASS
- authority local closure: `18/18` PASS
- authority external closure: `7/7` PASS

## Authority and backup integrity

Runtime authority v7:

- run id: `6becfb17a6b1dcb61446c0d4a866ed701b69f61e14856a2501760468047baa27`
- manifest SHA-256: `814a90cfca9e1c52c9ef2e536ac5b27f3bdb45bf21e318fcd3b807046fafc40d`
- commands SHA-256: `f5d2f2aff2da41142b36dfc4acc7dd5acb5eb8129caeccb4203c78dce0c0053f`
- authority directory mode: `0555`; manifest/commands mode and link count: `0400/1`

Preserved motion assets:

- right-handle executor SHA-256: `93bdb04608f0e31ebc064c6f0f6659a30e97d2c879d20e0b171e77499c0335cc`
- middle executor SHA-256: `e88dd07c1de79585c1e2534f870e0422394d227744ac2d4fafa0fc9267ab4246`
- return plan SHA-256: `24767afddc9101094a419d9ab00c04db352e344b1aae90b287d86ca4773ebb21`

Original assets remain unchanged:

- v6 manifest SHA-256: `de63865ea708810708b960ac827ded942882c87f006eb3a40019adb4f525726e`
- v6 commands SHA-256: `2964cc5feb54881f9c73b5d5b136342d0653c6b7766c483c50395a1ec8c143be`
- v5 manifest SHA-256: `d5fa37a982d3eb2d73c175a63c22efa52483e93ed973ccb02b8a2763a70d84ef`
- v5 commands SHA-256: `1ae170e78cee68f7b6f7ac7ec2ff29c1aaa91a9f35bba3809d806ad961d1c544`
- v1 manifest SHA-256: `e4ddd37824d44acb7aacf87e923d45a396b8a4b758c8fa3bd8b1577671cfb3db`
- v1 commands SHA-256: `12cd774a15257e8d7c75905dfe001c1ec48b87e0257456d8d8b9a885b6433a14`
- `831.tar.zst` SHA-256: `c2d1fdca42efd333d6c95e1f1ce183bd6449b48bfe7a38d1a258fef99a0f1ff4`
- 831 sidecar verification: `OK`
