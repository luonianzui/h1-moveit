#!/usr/bin/env bash
set -euo pipefail

# Real-vision tests for the 80 x 160 x 30 mm green block.  The cooperative
# mode starts with the block centered between the two arms; transfer modes use
# the right pickup area.  Gazebo, move_group and cube_detector must be running.

mode="${HANDOVER_MODE:-sequential_vision_handover}"
velocity_scaling="${VELOCITY_SCALING:-0.25}"
acceleration_scaling="${ACCELERATION_SCALING:-0.20}"
transfer_motion_velocity_scaling="${TRANSFER_MOTION_VELOCITY_SCALING:-0.20}"
transfer_motion_acceleration_scaling="${TRANSFER_MOTION_ACCELERATION_SCALING:-0.20}"
# The left receiver is empty and remains 100 mm above the block during this
# move.  Finish that safe pre-positioning promptly so the right hand does not
# have to support the airborne block for nearly a minute.
receiver_empty_velocity_scaling="${RECEIVER_EMPTY_VELOCITY_SCALING:-0.50}"
receiver_empty_acceleration_scaling="${RECEIVER_EMPTY_ACCELERATION_SCALING:-0.40}"
gripper_soft_contact="${GRIPPER_SOFT_CONTACT_POSITION:-0.001}"
pickup_test_lift="${PICKUP_TEST_LIFT:-0.05}"
cooperative_test_lift="${COOPERATIVE_TEST_LIFT:-0.050}"
if [[ "${mode}" == "sequential_center_regrasp_test" ||
      "${mode}" == "donor_release_test" ]]; then
  serial_carry_z_compensation="${SERIAL_CARRY_Z_COMPENSATION:-0.0}"
else
  serial_carry_z_compensation="${SERIAL_CARRY_Z_COMPENSATION:-0.0}"
fi
if [[ "${mode}" == "sequential_center_regrasp_test" ||
      "${mode}" == "donor_release_test" ||
      "${mode}" == "right_holder_place_recovery" ||
      "${mode}" == "receiver_place_recovery" ]]; then
  use_gazebo_grasp_constraint="${USE_GAZEBO_GRASP_CONSTRAINT:-true}"
else
  use_gazebo_grasp_constraint="${USE_GAZEBO_GRASP_CONSTRAINT:-false}"
fi
object_size_x="${OBJECT_SIZE_X:-0.080}"
object_size_y="${OBJECT_SIZE_Y:-0.160}"
object_size_z="${OBJECT_SIZE_Z:-0.030}"
right_grasp_y="${RIGHT_GRASP_Y_OFFSET:--0.064}"
if [[ "${mode}" == "sequential_center_regrasp_test" ||
      "${mode}" == "donor_release_test" ]]; then
  left_grasp_y="${LEFT_GRASP_Y_OFFSET:-0.008}"
else
  left_grasp_y="${LEFT_GRASP_Y_OFFSET:-0.035}"
fi
right_outward_yaw="${RIGHT_GRIPPER_OUTWARD_YAW_DEGREES:-0.0}"
left_outward_yaw="${LEFT_GRIPPER_OUTWARD_YAW_DEGREES:-0.0}"
# The 18.5 mm serial-donor height is the physically verified deeper grip.  At
# the fixed 60 degree inward tilt its exact open-hand mesh clearance above the
# table is still 10.541 mm.
grasp_z="${GRASP_Z_OFFSET:-0.0185}"
cooperative_grasp_y="${COOPERATIVE_GRASP_Y_OFFSET:-0.025}"
cooperative_grasp_z="${COOPERATIVE_GRASP_Z_OFFSET:-0.0205}"
cooperative_inward_tilt="${COOPERATIVE_INWARD_TILT_DEGREES:-60.0}"
cooperative_motion_duration="${COOPERATIVE_MOTION_DURATION:-3.00}"
cooperative_donor_approach_clearance="${COOPERATIVE_DONOR_APPROACH_CLEARANCE:-0.050}"
transfer_receiver_y="${TRANSFER_RECEIVER_Y_OFFSET:-0.025}"
transfer_receiver_z="${TRANSFER_RECEIVER_Z_OFFSET:-0.019114705}"
transfer_donor_x="${TRANSFER_DONOR_X_OFFSET:-0.0}"
transfer_receiver_tilt="${TRANSFER_RECEIVER_TILT_DEGREES:-80.0}"
transfer_receiver_slide_start_y="${TRANSFER_RECEIVER_SLIDE_START_Y:-0.060}"
transfer_receiver_staging_y="${TRANSFER_RECEIVER_STAGING_Y:-0.145}"
transfer_receiver_retreat="${TRANSFER_RECEIVER_RETREAT:-0.080}"
serial_receiver_prehandover_y="${SERIAL_RECEIVER_PREHANDOVER_Y_OFFSET:-0.115}"
if [[ "${mode}" == "sequential_vision_handover" ||
      "${mode}" == "sequential_center_regrasp_test" ||
      "${mode}" == "donor_release_test" ]]; then
  transfer_receiver_preload="${TRANSFER_RECEIVER_PRELOAD_POSITION:-0.003}"
else
  transfer_receiver_preload="${TRANSFER_RECEIVER_PRELOAD_POSITION:-0.009}"
fi
transfer_gate_observation="${TRANSFER_GATE_OBSERVATION:-1.0}"
transfer_gate_max_drift="${TRANSFER_GATE_MAX_DRIFT:-0.010}"
transfer_gate_max_anchor_displacement="${TRANSFER_GATE_MAX_ANCHOR_DISPLACEMENT:-0.010}"
transfer_gate_air_height_tolerance="${TRANSFER_GATE_AIR_HEIGHT_TOLERANCE:-0.010}"
transfer_gate_max_angle_degrees="${TRANSFER_GATE_MAX_ANGLE_DEGREES:-5.0}"
transfer_gate_max_tilt_degrees="${TRANSFER_GATE_MAX_TILT_DEGREES:-10.0}"
vision_grasp_x_correction="${VISION_GRASP_X_CORRECTION:-0.0}"
airborne_vision_x_correction="${AIRBORNE_VISION_X_CORRECTION:-0.002}"
airborne_vision_y_correction="${AIRBORNE_VISION_Y_CORRECTION:--0.010}"
airborne_vision_z_correction="${AIRBORNE_VISION_Z_CORRECTION:--0.002}"
spawn_x="${COOPERATIVE_SPAWN_X:-0.315}"
recovery_table_surface_z="${RECOVERY_TABLE_SURFACE_Z:-0.815}"
object_sdf="$(ros2 pkg prefix --share h1_2_description)/models/green_cube/model.sdf"

if [[ "${mode}" == "cooperative_lift_test" || "${mode}" == "cooperative_transfer_test" ]]; then
  gripper_preload="${GRIPPER_PRELOAD_POSITION:-0.003}"
elif [[ "${mode}" == "sequential_vision_handover" ||
        "${mode}" == "sequential_center_regrasp_test" ||
        "${mode}" == "donor_release_test" ]]; then
  gripper_preload="${GRIPPER_PRELOAD_POSITION:-0.003}"
else
  gripper_preload="${GRIPPER_PRELOAD_POSITION:-0.006}"
fi

case "${mode}" in
  right_holder_place_recovery|receiver_place_recovery|dual_hold_place_recovery|center_regrasp_abort_recovery)
    echo "[air-handover] Preserving the airborne block for safe placement recovery mode=${mode}..."
    ;;
  reset_only|pose_test)
    echo "[air-handover] Removing the block for the collision-aware dual-pose test..."
    gz service -s /world/h1_world/remove/blocking \
      --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 10000 \
      --req 'name: "green_cube", type: MODEL' || true
    ;;
  right_pick_test|cooperative_lift_test|cooperative_transfer_test|dual_contact_test|handover|sequential_vision_handover|sequential_center_regrasp_test|donor_release_test)
    spawn_y="${HANDOVER_SPAWN_Y:--0.180}"
    if [[ "${mode}" == "cooperative_lift_test" || "${mode}" == "cooperative_transfer_test" ]]; then
      spawn_y="${HANDOVER_SPAWN_Y:-0.00}"
    fi
    echo "[air-handover] Clearing the block before interrupted-state recovery..."
    gz service -s /world/h1_world/remove/blocking \
      --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 10000 \
      --req 'name: "green_cube", type: MODEL' || true
    echo "[air-handover] Parking both arms in an empty workspace before respawning the block..."
    ros2 run h1_2_moveit_demo bimanual_air_handover_demo --ros-args \
      -p mode:=reset_only \
      -p velocity_scaling:="${velocity_scaling}" \
      -p acceleration_scaling:="${acceleration_scaling}"
    echo "[air-handover] Recreating a stationary block at x=${spawn_x}, y=${spawn_y}..."
    create_result="$(gz service -s /world/h1_world/create/blocking \
      --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 10000 \
      --req "sdf_filename: \"${object_sdf}\", name: \"green_cube\", pose: {position: {x: ${spawn_x}, y: ${spawn_y}, z: 0.830}, orientation: {w: 1}}")"
    echo "${create_result}"
    if [[ "${create_result}" != *"data: true"* ]]; then
      # On a heavily loaded simulation the blocking reply can time out after
      # the server has already inserted the model.  Verify the resulting world
      # state before treating a missing acknowledgement as a hard failure.
      if timeout 5s gz model -m green_cube -p >/dev/null 2>&1; then
        echo "[air-handover] Create reply timed out, but green_cube exists; continuing."
      else
        echo "[air-handover] Failed to recreate the handover block; aborting." >&2
        exit 1
      fi
    fi
    ;;
  *)
    echo "[air-handover] HANDOVER_MODE must be right_holder_place_recovery, receiver_place_recovery, dual_hold_place_recovery, center_regrasp_abort_recovery, reset_only, pose_test, right_pick_test, cooperative_lift_test, cooperative_transfer_test, dual_contact_test, handover, sequential_vision_handover, sequential_center_regrasp_test or donor_release_test." >&2
    exit 2
    ;;
esac

echo "[air-handover] Running mode=${mode} at empty-space ${velocity_scaling}/${acceleration_scaling}, transfer ${transfer_motion_velocity_scaling}/${transfer_motion_acceleration_scaling}..."
ros2 run h1_2_moveit_demo bimanual_air_handover_demo --ros-args \
  -p mode:="${mode}" \
  -p velocity_scaling:="${velocity_scaling}" \
  -p acceleration_scaling:="${acceleration_scaling}" \
  -p transfer_motion_velocity_scaling:="${transfer_motion_velocity_scaling}" \
  -p transfer_motion_acceleration_scaling:="${transfer_motion_acceleration_scaling}" \
  -p receiver_empty_velocity_scaling:="${receiver_empty_velocity_scaling}" \
  -p receiver_empty_acceleration_scaling:="${receiver_empty_acceleration_scaling}" \
  -p gripper_soft_contact_position:="${gripper_soft_contact}" \
  -p gripper_preload_position:="${gripper_preload}" \
  -p object_size_x:="${object_size_x}" \
  -p object_size_y:="${object_size_y}" \
  -p object_size_z:="${object_size_z}" \
  -p pickup_test_lift:="${pickup_test_lift}" \
  -p serial_carry_z_compensation:="${serial_carry_z_compensation}" \
  -p use_gazebo_grasp_constraint:="${use_gazebo_grasp_constraint}" \
  -p cooperative_test_lift:="${cooperative_test_lift}" \
  -p right_grasp_y_offset:="${right_grasp_y}" \
  -p left_grasp_y_offset:="${left_grasp_y}" \
  -p right_gripper_outward_yaw_degrees:="${right_outward_yaw}" \
  -p left_gripper_outward_yaw_degrees:="${left_outward_yaw}" \
  -p grasp_z_offset:="${grasp_z}" \
  -p cooperative_grasp_y_offset:="${cooperative_grasp_y}" \
  -p cooperative_grasp_z_offset:="${cooperative_grasp_z}" \
  -p cooperative_inward_tilt_degrees:="${cooperative_inward_tilt}" \
  -p cooperative_motion_duration:="${cooperative_motion_duration}" \
  -p cooperative_donor_approach_clearance:="${cooperative_donor_approach_clearance}" \
  -p transfer_receiver_y_offset:="${transfer_receiver_y}" \
  -p transfer_receiver_z_offset:="${transfer_receiver_z}" \
  -p transfer_donor_x_offset:="${transfer_donor_x}" \
  -p transfer_receiver_tilt_degrees:="${transfer_receiver_tilt}" \
  -p transfer_receiver_slide_start_y:="${transfer_receiver_slide_start_y}" \
  -p transfer_receiver_staging_y:="${transfer_receiver_staging_y}" \
  -p transfer_receiver_retreat:="${transfer_receiver_retreat}" \
  -p serial_receiver_prehandover_y_offset:="${serial_receiver_prehandover_y}" \
  -p transfer_receiver_preload_position:="${transfer_receiver_preload}" \
  -p transfer_gate_observation:="${transfer_gate_observation}" \
  -p transfer_gate_max_drift:="${transfer_gate_max_drift}" \
  -p transfer_gate_max_anchor_displacement:="${transfer_gate_max_anchor_displacement}" \
  -p transfer_gate_air_height_tolerance:="${transfer_gate_air_height_tolerance}" \
  -p transfer_gate_max_angle_degrees:="${transfer_gate_max_angle_degrees}" \
  -p transfer_gate_max_tilt_degrees:="${transfer_gate_max_tilt_degrees}" \
  -p recovery_table_surface_z:="${recovery_table_surface_z}" \
  -p vision_grasp_x_correction:="${vision_grasp_x_correction}" \
  -p airborne_vision_x_correction:="${airborne_vision_x_correction}" \
  -p airborne_vision_y_correction:="${airborne_vision_y_correction}" \
  -p airborne_vision_z_correction:="${airborne_vision_z_correction}"

if [[ "${mode}" != "reset_only" && "${mode}" != "pose_test" ]]; then
  echo "[air-handover] Final Gazebo object state:"
  gz model -m green_cube -p
fi
