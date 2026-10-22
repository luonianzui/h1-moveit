#!/usr/bin/env bash
set -euo pipefail
set -C
workspace="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws"
root="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws/runtime_evidence/centrifuge_48box_sequential_full_20261022_r350"
review="/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws/runtime_evidence/centrifuge_48box_sequential_review_20261022_r8"
cd "$workspace"
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
export ROS_DOMAIN_ID=158 GZ_PARTITION=h1_seq_full_r350 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
test -n "${H1_TRACE_ARCHIVE_REGISTRY:-}"
test -f "$root/run/independent_first_frame_review.json"
test ! -e "$root/run/execution.claim"
bash "$root/run_dev.sh" "$root/startup_review_freeze.py" resume > "$review/resume_stdout.log" 2>&1 &
launcher=$!
deadline=$((SECONDS+60))
until test -f "$root/run/process_identities.json"; do
    kill -0 "$launcher"
    test "$SECONDS" -lt "$deadline"
    sleep 1
done
/usr/bin/python3 "$review/capture_native_identity.py" > "$review/native_identity.log" 2>&1
/usr/bin/python3 "$review/observe_runtime.py" observe > "$review/cpu_observer.log" 2>&1 &
cpu=$!
/usr/bin/python3 "$review/observe_server_threads.py" > "$review/thread_observer.log" 2>&1 &
threads=$!
bash "$root/run_dev.sh" "$review/observe_world_stats.py" > "$review/world_observer.log" 2>&1 &
world=$!
bash "$root/post_speed.sh" > "$root/post_speed.log" 2>&1 &
post=$!
set +e
wait "$launcher"; launch_status=$?
wait "$cpu"; cpu_status=$?
wait "$threads"; thread_status=$?
wait "$world"; world_status=$?
wait "$post"; post_status=$?
set -e
printf 'CONTINUOUS_COMPLETE launcher=%s cpu=%s threads=%s world=%s post=%s\n' "$launch_status" "$cpu_status" "$thread_status" "$world_status" "$post_status"
test "$launch_status" -eq 0
test "$post_status" -eq 0


