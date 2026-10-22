# Candidate-3 831 round-trip r17 live result

Status: **PASS**

This run used the non-overwriting v9 authority derived from the 901/v7
authority.  The complete two-cycle Gazebo flow executed visibly:

1. right hand opens the locked door;
2. left hand moves the cube from its origin into the incubator;
3. right hand closes and locks the door;
4. right hand opens the door again;
5. left hand returns the cube to its original table pose;
6. right hand closes and locks the door again.

The left hand never commands or contacts the door handle.

## Authority

- Authority: `candidate3_roundtrip_831_authority_v9`
- Run id: `c7813b59713f6b58a42a93b2d41cc7367096ae8607cb5229c46b2b6e8d2e6d4b`
- Manifest SHA-256: `b9e49b32c60979eb5a97d8ed3d5dc325a5d73e10b1985a45fd99c61f8848cd29`
- Commands SHA-256: `aef171adfacb8b1af212caa1c193e8fabf51e06d03f8fff4ceddf9ed0a22e8fb`
- Right executor SHA-256: `7a1233641c26298a92b0e153328dfd76e5e561a2f5520fec40f1c11a165ecde8`
- Controller authorization SHA-256: `6108f94f348e7d88ee96d94e587e7958bc52ec825fd1289803fb9b1bf8b5cb8c`

## Runtime timing

Monotonic markers:

- flow invocation: `87976469620453 ns`
- motion window start: `87979121651410 ns`
- motion window end: `88140993410423 ns`
- terminal completion PASS: `88140993455843 ns`

Derived wall durations:

- invocation to motion start: `2.652030957 s`
- motion window: `161.871759013 s`
- invocation to terminal PASS: `164.523835390 s`
- 901/r15 invocation to terminal PASS: `168.759167141 s`
- improvement over 901/r15: `4.235331751 s` (`2.509690%`)
- requirement: `< 180 s`; margin: `15.476164610 s`

## Speed optimization

Every v7/901 waypoint and all 60 controller-message positions are retained.
All loaded-cube, table-contact, support, stability, endpoint, and handle-contact
arc timings are unchanged.  Only unloaded right-side approach/egress and
gripper open/close point intervals are retimed:

- left launch/park seam: `0.82`
- right ingress/egress: `0.82`
- right gripper close/release: `0.87`
- both handle-contact arcs: `1.00` (901 timing)
- every middle/cube stage: unchanged

The authorization formally binds a `0.004 s` base point interval and a
`0.00328 s` minimum scheduled interval.  Offline and live execution share the
same scheduler and analytic Hermite validation.  Peak analytic use is
`38.314176%` of the velocity limit and `93.750000%` of the acceleration limit.

## Rosbag audit

Bag: `rosbag_831/rosbag_831_0.db3`

- total controller trajectory messages: `60`
- torso controller: `16`
- left arm controller: `16`
- right arm controller: `16`
- left gripper controller: `4`
- right gripper controller: `8`
- maximum absolute torso angle: `44.250000001 deg` (`< 45 deg`)
- `/incubator/handle/contacts` messages: `0`
- all serialized trajectory values finite: PASS

The recorder reported two transport-layer drops among 173,011 high-rate bag
messages.  All 60 required controller messages are present with exact topic
counts; the independent online torso/endpoint watchdogs and final fail-closed
gates also passed.

## Final Gazebo state

- cube position: `(0.334012987513, 0.377012427219, 0.829999537355) m`
- cube origin position error: `0.000017981249 m` (`0.017981249 mm`)
- cube orientation error: `0.000472799998 rad` (`0.02709 deg`)
- door hinge position: `-2.32512198383e-22 rad`
- handle position: `-2.85269448037e-16 rad`
- latch: `locked`
- door support: `detached`
- cube support: `detached`
- final robot state: launch pose
- left-handle commands: `0`
- handle-contact messages: `0`
- final publishers on all five trajectory topics: `0`

Gazebo remains running at the successful final state.  The close diagonal
view is saved as `final_state_best.png`.
