# Candidate-3 r30 pre-motion abort

Status: **ABORTED BEFORE PREPARATION OR MOTION**

The fresh `v20/r30` launch was stopped normally with `SIGINT` after the user
clarified that the complete double-cycle replay must run in the combined
Mid360 navigation-map scene, not the action authority's standalone world.

- No rosbag recorder was started.
- No support daemon was started and no support socket was created.
- No preparation command was invoked.
- No flow command was invoked.
- No controller trajectory command was issued by this run.
- No grip, support attach/detach, reset, door, handle, or cube operation ran.
- The launch was stopped only while the robot, incubator, and cube remained at
  their fresh initial state.
- `v20` and `r30` are consumed identities and must not be reused.

Post-stop checks found no Gazebo, replay, support-daemon, recorder, or flow
processes. Protected v9/r17, v19/r29, incubator-model, and backup-914 hashes
still match their recorded values.
