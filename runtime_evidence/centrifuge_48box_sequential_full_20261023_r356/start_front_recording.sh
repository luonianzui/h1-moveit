#!/usr/bin/env bash
set -eo pipefail
set -C
workspace="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws"
evidence="${workspace}/runtime_evidence/centrifuge_48box_sequential_full_20261023_r356"
cd "${workspace}"
export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
export ROS_DOMAIN_ID=164 GZ_PARTITION=h1_seq_full_r356
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4
test -n "${H1_TRACE_ARCHIVE_REGISTRY:-}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source install_lab_scene_48box_sequential_full_20261023_r356/local_setup.bash
test -f "${evidence}/identity.json"
test -f "${evidence}/full_window_capture_audit.json"
test ! -e "${evidence}/run"
mkdir "${evidence}/run"
/usr/bin/python3 "${evidence}/fresh_original_source_check.py" > "${evidence}/run/fresh_original_source_check.log" 2>&1
/usr/bin/python3 "${evidence}/launch_scene.py" > "${evidence}/run/startup_launch_scene.log" 2>&1
/usr/bin/python3 "${evidence}/controller_health.py" > "${evidence}/run/startup_controller_health.log" 2>&1
/usr/bin/python3 "${evidence}/verify_dart_runtime.py" > "${evidence}/run/startup_verify_dart_runtime.log" 2>&1
/usr/bin/python3 "${evidence}/audit_transport_environment.py" > "${evidence}/run/startup_transport_environment.log" 2>&1
/usr/bin/python3 "${evidence}/initialize_stationary_legs.py" "${evidence}/run/leg_initialization.json" > "${evidence}/run/leg_initialization.log" 2>&1
/usr/bin/python3 "${evidence}/read_startup_ramp.py" > "${evidence}/run/startup_read_startup_ramp.log" 2>&1
/usr/bin/python3 "${evidence}/install_profile_plugin.py" > "${evidence}/run/startup_install_profile_plugin.log" 2>&1
/usr/bin/python3 "${evidence}/startup_review_freeze.py" freeze > "${evidence}/run/startup_review_freeze.log" 2>&1
/usr/bin/python3 "${evidence}/video_smoke.py" > "${evidence}/run/startup_video_smoke.log" 2>&1
/usr/bin/python3 "${evidence}/start_full_window_video.py" > "${evidence}/run/startup_start_full_window_video.log" 2>&1
/usr/bin/python3 "${evidence}/review_first_frame.py" > "${evidence}/run/startup_frame_decode.log" 2>&1
printf 'FRONT_RECORDING_STARTED_REQUIRES_ACTUAL_FIRST_FRAME_REVIEW_BEFORE_EXECUTOR\n'
