#!/usr/bin/python3
"""Regression tests for the 8254 mechanism settling gate."""

from pathlib import Path
import subprocess
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
FULL_WRAPPER = PACKAGE_ROOT / "scripts" / "h1_full_incubator_8251.sh"
FAST_WRAPPER = PACKAGE_ROOT / "scripts" / "h1_closed_loop_90_8254.sh"


class MechanismGateContractTest(unittest.TestCase):
    @staticmethod
    def function_source(source: str, name: str, next_name: str) -> str:
        start = source.index(f"{name}()")
        end = source.index(f"{next_name}()", start)
        return source[start:end]

    def run_gate(self, snapshot_body: str, expect_pass: bool) -> subprocess.CompletedProcess[str]:
        expected_branch = "exit 0" if expect_pass else "exit 90"
        rejected_branch = "exit 91" if expect_pass else "exit 0"
        command = f"""
source {FULL_WRAPPER}
trap - EXIT INT TERM HUP
read_incubator_snapshot() {{ {snapshot_body}; }}
if ( mechanism_gate TEST 0.0 0.01 0.0 0.01 ); then
  {expected_branch}
else
  {rejected_branch}
fi
"""
        return subprocess.run(
            ["/bin/bash", "-c", command],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def test_failure_propagates_when_called_from_if_condition(self) -> None:
        cases = (
            "return 1",
            "printf '%s\\n' 'handle_latch_joint 0.0 0.0'",
            "printf '%s\\n' 'door_hinge_joint 0.1 0.0' 'handle_latch_joint 0.0 0.0'",
            "printf '%s\\n' 'door_hinge_joint 0.0 0.003' 'handle_latch_joint 0.0 0.0'",
        )
        for snapshot_body in cases:
            with self.subTest(snapshot_body=snapshot_body):
                result = self.run_gate(snapshot_body, expect_pass=False)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertNotIn("TEST=PASS", result.stdout)

    def test_valid_stationary_snapshot_passes(self) -> None:
        result = self.run_gate(
            "printf '%s\\n' 'door_hinge_joint 0.0 0.0' "
            "'handle_latch_joint 0.0 0.0'",
            expect_pass=True,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("TEST=PASS", result.stdout)

    def test_fast_wrapper_bounds_each_snapshot_and_retry_count(self) -> None:
        source = FAST_WRAPPER.read_text(encoding="utf-8")
        self.assertIn(
            'timeout 0.2 "${ROS_SNAPSHOT_TOOL}" incubator --timeout 0.1',
            source,
        )
        self.assertIn("for attempt in $(seq 1 11)", source)
        self.assertIn("(( attempt == 11 )) || sleep 0.05", source)
        self.assertIn('local label="$1" attempt last_log\n', source)
        self.assertIn('last_log="${RUN_DIR}/settle_${label}.last.log"', source)
        self.assertIn("export H1_CUBE_RACK_POSITION_TOLERANCE=0.030", source)
        self.assertIn(
            "readonly cube_parallel_yaw_rad='1.570796326794897'",
            source,
        )
        self.assertIn(
            "readonly cube_parallel_orientation_tolerance_rad="
            "'0.034906585039887'",
            source,
        )
        self.assertIn("export H1_CUBE_RACK_EXPECTED_YAW=90", source)
        self.assertIn(
            'export H1_CUBE_RACK_ORIENTATION_TOLERANCE="'
            '${cube_parallel_orientation_tolerance_rad}"',
            source,
        )
        self.assertNotIn("1.221730476396031", source)
        self.assertNotIn("0.523598775598299", source)
        self.assertEqual(source.count('"${cube_parallel_yaw_rad}"'), 7)
        self.assertEqual(
            source.count('"${cube_parallel_orientation_tolerance_rad}"'),
            8,
        )
        self.assertIn("export H1_CUBE_ARM_ENDPOINT_TOLERANCE=0.012", source)
        self.assertIn(
            '"${CUBE_SUPPORTED_CARRY_TIME_SCALES}" "${cube_payload_sha}" 0.012 0.012',
            source,
        )
        self.assertIn("fast_replay cube_release", source)
        self.assertIn("settled_cube_release_gate CUBE_RELEASED_ON_MIDDLE_RACK", source)
        self.assertIn("fast_replay cube_retreat", source)
        self.assertIn("export H1_CLOSE_SKIP_PRE_TRANSPORT=1", source)
        self.assertIn("export H1_CLOSE_SKIP_POST_TRANSPORT=1", source)
        self.assertIn("settled_left_handle_hold_gate LEFT_REGRASP_HOLD_GATE", source)
        self.assertIn("readonly cube_retreat_minimum_rtf='0.55'", source)
        self.assertIn(
            'CUBE_RETREAT_PROFILE \\\n    "${cube_retreat_minimum_rtf}" 1',
            source,
        )
        self.assertIn(
            "export H1_FULL_CUBE_BEFORE_ATTACH_TIME_SCALES="
            "'0:0.36,1:0.08'",
            source,
        )
        self.assertIn(
            "export H1_FULL_CUBE_SUPPORTED_CARRY_TIME_SCALES="
            "'2:0.36,3:0.36'",
            source,
        )
        self.assertIn(
            "export H1_FULL_CUBE_AFTER_DETACH_TIME_SCALES="
            "'4:0.08,5:0.36'",
            source,
        )
        self.assertIn("readonly cube_release_scale='4:0.08'", source)
        self.assertIn("readonly cube_retreat_scale='5:0.36'", source)
        self.assertIn(
            'readonly cube_asset="${parallel_cube_root}/'
            'cube.parallel90.neutral.trajectory.json"',
            source,
        )
        self.assertIn(
            "readonly cube_payload_sha="
            "'acaec97384f16a736f6b246a3fa1a0897b7aad93e72a6204701610ebf4f7e705'",
            source,
        )
        self.assertNotIn("left_neutral_bridge", source)
        self.assertNotIn("fast_replay left_neutral_bridge", source)
        self.assertEqual(
            source.count(
                "H1_DOOR_SUPPORT_FAST_RESIDENT=1 run_door_support_hook"
            ),
            3,
        )
        self.assertEqual(
            source.count(
                "H1_CUBE_SUPPORT_FAST_RESIDENT=1 run_cube_support_hook attach"
            ),
            2,
        )
        self.assertEqual(
            source.count(
                "H1_CUBE_SUPPORT_FAST_RESIDENT=1 run_cube_support_hook detach"
            ),
            2,
        )
        self.assertNotIn(
            "H1_DOOR_SUPPORT_FAST_RESIDENT=1 run_door_support_preload",
            source,
        )
        self.assertNotIn(
            "H1_CUBE_SUPPORT_FAST_RESIDENT=1 run_cube_support_preload",
            source,
        )

        reverse = self.function_source(
            source,
            "run_reverse_sequence",
            "run_prepare_q60_sequence",
        )
        door_close = reverse.index("fast_replay door_close")
        relock_rotate = reverse.index(
            'fast_replay handle_relock_rotate "${handle_relock_asset}" handle 0 1'
        )
        down_while_clamped = reverse.index(
            "handle_vertical_down_while_clamped_gate", relock_rotate
        )
        relock_release = reverse.index(
            'fast_replay handle_relock_release_retreat '
            '"${handle_relock_asset}" handle 1 5',
            down_while_clamped,
        )
        motion_end = reverse.index(
            'FLOW_END_NS="$(monotonic_now_ns)"', relock_release
        )
        handle_down = reverse.index("handle_vertical_down_gate", motion_end)
        relock_deadline = reverse.index(
            "deadline_gate HANDLE_RELOCK_DEADLINE",
            handle_down,
        )
        self.assertLess(door_close, relock_rotate)
        self.assertLess(relock_rotate, down_while_clamped)
        self.assertLess(down_while_clamped, relock_release)
        self.assertLess(relock_release, motion_end)
        self.assertLess(motion_end, handle_down)
        self.assertLess(handle_down, relock_deadline)

        full = self.function_source(
            source,
            "run_full_sequence",
            "cleanup_8254",
        )
        cube_retreat = full.index("fast_replay cube_retreat")
        retreat_gate = full.index(
            "run_parallel_gate_group CUBE_RETREAT_DONE", cube_retreat
        )
        reverse_start = full.index("run_reverse_sequence", retreat_gate)
        self.assertLess(cube_retreat, retreat_gate)
        self.assertLess(retreat_gate, reverse_start)
        self.assertNotIn("fast_replay left_neutral_bridge", full)

        handle_down_gate = self.function_source(
            source,
            "handle_vertical_down_gate",
            "run_detach_at_q0",
        )
        self.assertIn("run_parallel_gate_group HANDLE_VERTICAL_DOWN", handle_down_gate)
        self.assertIn("${DOOR_ZERO_RAD}", handle_down_gate)
        self.assertIn("${HANDLE_ZERO_RAD}", handle_down_gate)
        self.assertIn("0.087266462599716", handle_down_gate)
        self.assertIn(
            "require_latch_state locked HANDLE_VERTICAL_DOWN_LATCH",
            handle_down_gate,
        )
        self.assertIn(
            "HANDLE_VERTICAL_DOWN=PASS door=0deg handle=0deg latch=locked",
            handle_down_gate,
        )

        clamped_down_gate = self.function_source(
            source,
            "handle_vertical_down_while_clamped_gate",
            "run_detach_at_q0",
        )
        self.assertIn(
            "run_parallel_gate_group HANDLE_DOWN_WHILE_CLAMPED",
            clamped_down_gate,
        )
        self.assertIn("${DOOR_ZERO_RAD}", clamped_down_gate)
        self.assertIn("${HANDLE_ZERO_RAD}", clamped_down_gate)
        self.assertIn(
            "require_latch_state locked HANDLE_DOWN_WHILE_CLAMPED_LATCH",
            clamped_down_gate,
        )
        self.assertIn(
            "left_handle_hold_gate HANDLE_DOWN_WHILE_CLAMPED_LEFT_HOLD",
            clamped_down_gate,
        )

        global_relock = source.index("fast_replay handle_relock_release_retreat")
        global_motion_end = source.index(
            'FLOW_END_NS="$(monotonic_now_ns)"',
            global_relock,
        )
        final_gate = source.index(
            "run_parallel_gate_group FINAL_PHYSICAL",
            global_motion_end,
        )
        self.assertLess(global_motion_end, final_gate)
        self.assertIn(
            '"${cube_parallel_yaw_rad}" 0.015',
            source[final_gate:],
        )
        self.assertIn(
            '"${cube_parallel_orientation_tolerance_rad}"',
            source[final_gate:],
        )
        self.assertIn("post_motion_validation_seconds.txt", source)

    def test_resident_gazebo_value_caches_are_fresh_atomic_and_cleaned_up(self) -> None:
        source = FAST_WRAPPER.read_text(encoding="utf-8")
        self.assertIn(
            "readonly gazebo_state_cache_max_age_ns='500000000'", source
        )
        self.assertEqual(
            source.count(
                "start_gazebo_value_monitor /h1_2/right_door_support/state"
            ),
            1,
        )
        self.assertEqual(
            source.count("start_gazebo_value_monitor /incubator/latch/state"),
            1,
        )
        monitor = self.function_source(
            source, "start_gazebo_value_monitor", "read_cached_gazebo_value"
        )
        embedded_start = monitor.index("import os\n")
        embedded_end = monitor.index("\nPY\n", embedded_start)
        compile(monitor[embedded_start:embedded_end], str(FAST_WRAPPER), "exec")
        for token in (
            "sequence += 1",
            "receipt_ns = time.monotonic_ns()",
            "os.replace(temporary, cache)",
            '["gz", "topic", "-e", "-t", topic]',
            "time.sleep(0.05)",
        ):
            self.assertIn(token, monitor)

        reader = self.function_source(
            source, "read_cached_gazebo_value", "start_gazebo_state_monitors"
        )
        for token in (
            '"${schema}" == h1_gazebo_value_cache/v1',
            '"${sequence}" =~ ^[1-9][0-9]*$',
            '"${receipt}" =~ ^[1-9][0-9]*$',
            "(( receipt <= now ))",
            "(( age <= gazebo_state_cache_max_age_ns ))",
        ):
            self.assertIn(token, reader)

        support = self.function_source(
            source, "support_state_value", "support_state_gate"
        )
        latch_start = source.index("\nrequire_latch_state()\n")
        latch_end = source.index("\n# The resident monitor refreshes", latch_start)
        latch = source[latch_start:latch_end]
        self.assertIn("read_cached_gazebo_value", support)
        self.assertIn("read_cached_gazebo_value", latch)
        self.assertNotIn("gz topic", support + latch)
        self.assertIn("sleep 0.02", latch)

        cleanup = self.function_source(source, "cleanup_8254", "interrupt_8254")
        self.assertIn("stop_gazebo_state_monitors", cleanup)
        main_start = source.index("main_8254()")
        main = source[main_start : source.index("\ntrap cleanup_8254 EXIT", main_start)]
        self.assertLess(
            main.index("start_gazebo_state_monitors"),
            main.index("basic_health_gate"),
        )

        initial_monitors = self.function_source(
            source,
            "start_gazebo_state_monitors",
            "start_support_state_monitor",
        )
        support_monitor = self.function_source(
            source,
            "start_support_state_monitor",
            "stop_gazebo_state_monitors",
        )
        self.assertNotIn("/h1_2/right_door_support/state", initial_monitors)
        self.assertIn("/h1_2/right_door_support/state", support_monitor)
        self.assertIn(
            '[[ -z "${SUPPORT_STATE_MONITOR_PID}" ]]', support_monitor
        )

        for entry_name, next_name in (
            ("run_prepare_q60_sequence", "run_right_middle_sequence"),
            ("run_full_sequence", "cleanup_8254"),
        ):
            entry = self.function_source(source, entry_name, next_name)
            preload = entry.index("run_door_support_preload")
            monitor_start = entry.index("start_support_state_monitor", preload)
            self.assertLess(preload, monitor_start)

        reverse_branch = main[main.index('if [[ "${closed_loop_mode}" == reverse-run ]') :]
        self.assertLess(
            reverse_branch.index("start_support_state_monitor"),
            reverse_branch.index("q60_checkpoint REVERSE_ENTRY_Q60"),
        )

    def test_prepare_q60_is_a_door_only_stationary_collector_prefix(self) -> None:
        fast_source = FAST_WRAPPER.read_text(encoding="utf-8")
        prepare = self.function_source(
            fast_source,
            "run_prepare_q60_sequence",
            "run_right_middle_sequence",
        )

        # This engineering entry may prepare the exact supported q60 scene,
        # but it must never touch the cube or run any cleanup/recovery action.
        self.assertNotIn("run_cube_support_preload", prepare)
        self.assertNotIn("run_cube_support_hook", prepare)
        self.assertNotRegex(prepare, r"(?m)^\s*fast_replay\s+cube(?:_|\b)")

        executable = "\n".join(
            line
            for line in prepare.splitlines()
            if not line.lstrip().startswith(("#", "log "))
        ).lower()
        for forbidden_operation in ("detach", "release", "reset"):
            with self.subTest(forbidden_operation=forbidden_operation):
                self.assertNotIn(forbidden_operation, executable)

        # The collector must prove the exact live checkpoint before returning:
        # healthy real-time factor, stationary q60, left arm clear, bounded
        # duration, and no remaining robot trajectory publisher.
        required_gates = (
            "physics_and_rtf_gate PREPARE_Q60_RTF_GATE",
            "q60_checkpoint PREPARE_Q60_FINAL",
            "left_clear_gate PREPARE_Q60_LEFT_CLEAR",
            "deadline_gate PREPARE_Q60_DEADLINE",
            "stop_persistent_runner",
            "require_no_robot_trajectory_publishers",
        )
        for required_gate in required_gates:
            with self.subTest(required_gate=required_gate):
                self.assertIn(required_gate, prepare)

        full_source = FULL_WRAPPER.read_text(encoding="utf-8")
        publisher_gate = self.function_source(
            full_source,
            "require_no_robot_trajectory_publishers",
            "run_parallel_gate_group",
        )
        self.assertIn("'Publisher count: 0'", publisher_gate)
        self.assertIn("for index in \"${!ROBOT_TRAJECTORY_TOPICS[@]}\"", publisher_gate)
        topic_block_start = full_source.index("readonly -a ROBOT_TRAJECTORY_TOPICS=(")
        topic_block_end = full_source.index(")", topic_block_start)
        topic_block = full_source[topic_block_start:topic_block_end]
        expected_topics = (
            "/right_arm_controller/joint_trajectory",
            "/left_arm_controller/joint_trajectory",
            "/right_gripper_controller/joint_trajectory",
            "/left_gripper_controller/joint_trajectory",
        )
        configured_topics = tuple(
            line.strip()
            for line in topic_block.splitlines()[1:]
            if line.strip()
        )
        self.assertEqual(configured_topics, expected_topics)

    def test_right_middle_run_excludes_every_left_handle_asset(self) -> None:
        source = FAST_WRAPPER.read_text(encoding="utf-8")
        middle = self.function_source(
            source,
            "run_right_middle_sequence",
            "run_full_sequence",
        )
        reverse = self.function_source(
            source,
            "run_right_middle_reverse_sequence",
            "run_prepare_q60_sequence",
        )
        self.assertIn("fast_replay door_pre_support", middle)
        self.assertIn("fast_replay cube_contact", middle)
        self.assertIn("run_right_middle_reverse_sequence", middle)
        self.assertIn("fast_replay door_close", reverse)
        self.assertIn("fast_replay right_release", reverse)
        self.assertIn("fast_replay right_retreat", reverse)
        for forbidden in (
            "left_regrasp",
            "handle_relock",
            "left_handle_hold_gate",
            "handle_vertical_down_while_clamped_gate",
        ):
            self.assertNotIn(forbidden, middle)
            self.assertNotIn(forbidden, reverse)


if __name__ == "__main__":
    unittest.main()
