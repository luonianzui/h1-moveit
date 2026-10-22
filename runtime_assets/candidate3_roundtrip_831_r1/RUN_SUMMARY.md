# Candidate-3 831 round-trip live run r1

Status: **PASS** (Gazebo live execution, flow exit code 0)

- Authority run ID: `c2e930bc39f0539d9e307b85cb814f59a44ce3249febb16a2e498da77f949b89`
- Authority manifest SHA-256: `e4ddd37824d44acb7aacf87e923d45a396b8a4b758c8fa3bd8b1577671cfb3db`
- Commands SHA-256: `12cd774a15257e8d7c75905dfe001c1ec48b87e0257456d8d8b9a885b6433a14`
- Return plan SHA-256: `24767afddc9101094a419d9ab00c04db352e344b1aae90b287d86ca4773ebb21`
- Return authorization SHA-256: `7f9c868234344c881ad6e6b0a8e44d23bca41ea08ed2aff3d19dc3e9e803fb4a`

## Executed sequence

1. Right hand opened the door; left hand inserted the cube; right hand closed and locked the door.
2. The cycle-1 checkpoint passed with cube inside, door and handle at q0, latch locked, supports detached, and robot at launch.
3. Right hand reopened the door; left hand grasped the cube inside and returned it along the exact inverse path to the origin; right hand closed and relocked the door.
4. Finalization passed with cube at origin, door and handle at q0, latch locked, supports detached, robot at launch, and zero trajectory publishers.

## Measured evidence

- Motion window: `174.238048405 s` (`33425970956147` to `33600209004552` monotonic ns).
- Controller messages: `60` total: torso 16, right arm 16, left arm 16, right gripper 8, left gripper 4.
- Maximum absolute torso angle: `44.250000001 deg` (hard gate `44.5 deg`; user limit `45 deg`).
- Final door hinge: `-2.3251219838253058e-22 rad`.
- Final handle latch: `-2.8526944801767947e-16 rad`; latch topic: `locked`.
- Final cube pose, repeated for three stable samples: position `[0.334116, 0.377065, 0.829999] m`, RPY `[-0.000001, -0.000001, 1.571180] rad`.
- Cube origin error: `0.132974 mm`; yaw error: `0.021983 deg`.
- Final controller trajectory publisher counts: all five topics `0`.
- Support daemon protocol: all 11 requests passed, including cycle checkpoint, return attach/detach, finalization, and shutdown.

The left hand is never assigned a handle operation and the return execution authorization forbids left-hand handle contact. The native Gazebo handle-contact topic produced zero messages in this run, so this summary does not elevate that absence into a native-contact certificate; the return is instead the pointwise exact inverse of the already audited Candidate-3 path.

## Artifacts

- `rosbag_831/rosbag_831_0.db3`: SHA-256 `b479f84e0435c6ef8b87945abe72869054bc550a5044df2f8c84a7b491fa664a`
- `rosbag_831/metadata.yaml`: SHA-256 `69b74d0e8c3ec74c73ecee1431e1106a77e8bcc745fcc5920dd014d7745d1292`
- `final_gazebo.png`: SHA-256 `6597b2f87ab788f6343d3cb0af990210559a9cafa2811355b4ac5296b6016dc3`

Gazebo GUI was deliberately left running at the successful final state.
