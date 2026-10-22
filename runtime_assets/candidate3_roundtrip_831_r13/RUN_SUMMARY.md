# Candidate 3 roundtrip 831 r13 — verified run summary

Date: 2026-09-01 (Asia/Shanghai)

## Outcome

`PASS`

The live Gazebo run completed the requested two-cycle sequence:

1. Right hand opens the door.
2. Left hand inserts the cube.
3. Right hand closes and locks the door.
4. Right hand opens the door again.
5. Left hand retrieves the cube to its sealed origin pose.
6. Right hand closes and locks the door again.

Terminal verdict:

```text
CANDIDATE3_ROUNDTRIP_831=PASS cycles=2 cube=origin door=locked left_handle_commands=0 left_handle_contact_forbidden=true
```

Motion window:

- start monotonic ns: `6948624649962`
- end monotonic ns: `7114164485926`
- total: `165.539835964 s`
- average per cycle: `82.769917982 s`
- requirement: `< 90 s/cycle` — PASS

## Motion and safety evidence

The rosbag contains exactly 60 controller trajectory messages:

- torso: `16`
- left arm: `16`
- right arm: `16`
- left gripper: `4`
- right gripper: `8`

Maximum measured absolute torso angle from `/joint_states`:

- `0.7723081940252895 rad`
- `44.250000001019792 deg`
- user limit: `45 deg` — PASS

Handle evidence:

- flow validator: `left_handle_commands=0`
- `/incubator/handle/contacts`: `0` recorded messages
- the generic contact topic is not a native left-vs-right contact certificate; the semantic left-hand prohibition is enforced by the sealed flow/controller contract and terminal validator.

## Verified final live state

- door position: `-2.3251219838253058e-22 rad`
- handle position: `-2.8526944802655657e-16 rad`
- latch: `locked`
- cube world position: `(0.33415500116309776, 0.376981877930732, 0.82999940034140474) m`
- sealed cube origin: `(0.334, 0.377, 0.830) m`
- cube origin translation error: `0.156058 mm`
- active trajectory publishers after completion: `0` on all five controller topics
- Gazebo GUI remains running at the successful final state.

## Recording

Trimmed demonstration (preload and post-run wait removed):

- file: `candidate3_roundtrip_831_r13_demo.mp4`
- H.264 / yuv420p / `1160x1000` / constant `6 fps`
- duration: `177.000000 s`
- frames: `1062`
- SHA-256: `d56722f837cc6c22b03cb9deaeb6222c02ab68fc3be8ce4704532923036bf8e8`
- full decode: PASS, zero ffmpeg errors

Untrimmed audit recording:

- file: `candidate3_roundtrip_831_r13_close.mp4`
- H.264 / yuv420p / `1160x1000` / constant `6 fps`
- duration: `349.333333 s`
- frames: `2096`
- SHA-256: `7affb1c990938d41862607a0f3e061cc288a22eb3723fa60c939031b73a1f16b`
- full decode: PASS, zero ffmpeg errors
- visual inspection confirmed the complete two-cycle sequence with no menu, terminal, or dialog obstruction.

Rosbag:

- path: `rosbag_831/`
- duration: `346.186525050 s`
- total messages: `169289`
- all 60 controller trajectory messages are present.
- recorder reported two transport-layer telemetry sample losses on shutdown; controller counts, terminal validators, and final live-state checks are complete.

## Authority and backup integrity

Runtime authority v5:

- manifest SHA-256: `d5fa37a982d3eb2d73c175a63c22efa52483e93ed973ccb02b8a2763a70d84ef`
- commands SHA-256: `1ae170e78cee68f7b6f7ac7ec2ff29c1aaa91a9f35bba3809d806ad961d1c544`
- closure: `18/18` local and `7/7` external bindings matched
- authority directory mode: `0555`; manifest mode/link count: `0400/1`

Original assets remain unchanged:

- v1 manifest SHA-256: `e4ddd37824d44acb7aacf87e923d45a396b8a4b758c8fa3bd8b1577671cfb3db`
- v1 commands SHA-256: `12cd774a15257e8d7c75905dfe001c1ec48b87e0257456d8d8b9a885b6433a14`
- `831.tar.zst` SHA-256: `c2d1fdca42efd333d6c95e1f1ce183bd6449b48bfe7a38d1a258fef99a0f1ff4`
- 831 sidecar verification: `OK`

Pre-run related test suite: `150 passed`.
