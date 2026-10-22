#!/usr/bin/env bash
set -euo pipefail

# Real-vision tabletop relay for the 80 x 160 x 30 mm green block:
# right hand -> central handoff point -> left hand.
# Gazebo, move_group and the real cube_detector must already be running.

velocity_scaling="${VELOCITY_SCALING:-0.50}"
acceleration_scaling="${ACCELERATION_SCALING:-0.50}"

echo "[relay] Parking the right arm to clear the initial camera view..."
ros2 run h1_2_moveit_demo arm_pick_place_vision_demo --ros-args \
  -p arm_side:=right \
  -p park_only:=true \
  -p velocity_scaling:="${velocity_scaling}" \
  -p acceleration_scaling:="${acceleration_scaling}"

echo "[relay] Recreating the green handover block at the right-hand pickup area..."
object_sdf="$(ros2 pkg prefix --share h1_2_description)/models/green_cube/model.sdf"
gz service -s /world/h1_world/remove/blocking \
  --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 10000 \
  --req 'name: "green_cube", type: MODEL' || true
create_result="$(gz service -s /world/h1_world/create/blocking \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 10000 \
  --req "sdf_filename: \"${object_sdf}\", name: \"green_cube\", pose: {position: {x: 0.30, y: -0.25, z: 0.830}, orientation: {w: 1}}")"
echo "${create_result}"
if [[ "${create_result}" != *"data: true"* ]]; then
  echo "[relay] Failed to recreate the green handover block; aborting." >&2
  exit 1
fi

echo "[relay] Phase 1/2: right hand picks and places at the central handoff point."
ros2 run h1_2_moveit_demo arm_pick_place_vision_demo --ros-args \
  -p arm_side:=right \
  -p place_x:=0.30 \
  -p place_y:=0.0 \
  -p park_after_retreat:=true \
  -p detection_timeout:=10.0 \
  -p velocity_scaling:="${velocity_scaling}" \
  -p acceleration_scaling:="${acceleration_scaling}"

echo "[relay] Phase 2/2: left hand re-detects the cube and carries it left."
ros2 run h1_2_moveit_demo arm_pick_place_vision_demo --ros-args \
  -p arm_side:=left \
  -p place_x:=0.25 \
  -p place_y:=0.35 \
  -p detection_timeout:=10.0 \
  -p velocity_scaling:="${velocity_scaling}" \
  -p acceleration_scaling:="${acceleration_scaling}"

echo "[relay] Bimanual tabletop relay completed."
gz model -m green_cube -p
