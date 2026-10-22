#!/usr/bin/env python3
"""Pure-source contract for the q0 support detach helper."""

from pathlib import Path
import subprocess
import unittest


HELPER = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "h1_door_support_detach_8253.sh"
)
SOURCE = HELPER.read_text(encoding="utf-8")


def function_body(name: str, next_name: str) -> str:
    start = SOURCE.index(f"\n{name}()\n")
    function_marker = f"\n{next_name}()\n"
    statement_marker = f"\n{next_name}\n"
    if function_marker in SOURCE[start:]:
        end = SOURCE.index(function_marker, start)
    else:
        end = SOURCE.index(statement_marker, start)
    return SOURCE[start:end]


class DoorSupportDetach8253SourceContract(unittest.TestCase):
    def test_bash_parser_accepts_helper(self) -> None:
        subprocess.run(["bash", "-n", str(HELPER)], check=True)

    def test_optional_snapshot_tool_is_an_absolute_executable(self) -> None:
        self.assertIn(
            'readonly ros_snapshot_tool="${H1_CLOSE_ROS_SNAPSHOT_TOOL:-}"',
            SOURCE,
        )
        self.assertIn('if [[ -n "${ros_snapshot_tool}" ]]; then', SOURCE)
        self.assertIn(
            '[[ "${ros_snapshot_tool}" == /* && -f "${ros_snapshot_tool}" &&',
            SOURCE,
        )
        self.assertIn('-x "${ros_snapshot_tool}" ]]', SOURCE)
        self.assertIn(
            "H1_CLOSE_ROS_SNAPSHOT_TOOL must name an absolute executable", SOURCE
        )
        self.assertIn(
            'readonly skip_post_transport="${H1_CLOSE_SKIP_POST_TRANSPORT:-0}"',
            SOURCE,
        )
        self.assertIn(
            'readonly skip_pre_transport="${H1_CLOSE_SKIP_PRE_TRANSPORT:-0}"',
            SOURCE,
        )
        self.assertIn('[[ "${skip_pre_transport}" == 0 ||', SOURCE)
        self.assertIn('[[ "${skip_post_transport}" == 0 ||', SOURCE)

    def test_robot_snapshot_precedes_unchanged_ros_fallback(self) -> None:
        body = function_body("robot_rows", "mechanism_rows")
        fallback = 'sample="$(timeout 8 ros2 topic echo --once /joint_states 2>/dev/null)"'
        fallback_at = body.index(fallback)
        snapshot = body[:fallback_at]
        self.assertIn("for attempt in 1 2 3; do", snapshot)
        self.assertIn(
            'rows="$(timeout 5 "${ros_snapshot_tool}" robot --timeout 3)"',
            snapshot,
        )
        self.assertIn("printf '%s\\n' \"${rows}\"", snapshot)
        self.assertIn("return 1", snapshot)
        self.assertNotIn("ros2 topic echo", snapshot)
        self.assertEqual(body.count(fallback), 1)

    def test_incubator_snapshot_precedes_unchanged_ros_fallback(self) -> None:
        body = function_body("mechanism_rows", "require_robot_close_endpoint")
        fallback = (
            'sample="$(timeout 8 ros2 topic echo --once '
            '/incubator/joint_states 2>/dev/null)"'
        )
        fallback_at = body.index(fallback)
        snapshot = body[:fallback_at]
        self.assertIn("for attempt in 1 2 3; do", snapshot)
        self.assertIn(
            'rows="$(timeout 5 "${ros_snapshot_tool}" incubator --timeout 3)"',
            snapshot,
        )
        self.assertIn("printf '%s\\n' \"${rows}\"", snapshot)
        self.assertIn("return 1", snapshot)
        self.assertNotIn("ros2 topic echo", snapshot)
        self.assertEqual(body.count(fallback), 1)
        for token in (
            'section=="name" && /^- / {name[++n]=$2; next}',
            'section=="position" && /^- / {position[++p]=$2; next}',
            'section=="velocity" && /^- / {velocity[++v]=$2; next}',
            "if(n==0 || p!=n || v!=n) exit 1",
            "print name[i],position[i],velocity[i]",
        ):
            self.assertIn(token, body)

    def test_snapshot_failure_cannot_fall_through_to_ros_cli(self) -> None:
        for body, fallback in (
            (
                function_body("robot_rows", "mechanism_rows"),
                "ros2 topic echo --once /joint_states",
            ),
            (
                function_body("mechanism_rows", "require_robot_close_endpoint"),
                "ros2 topic echo --once /incubator/joint_states",
            ),
        ):
            snapshot_return = body.index("\n    return 1\n")
            fallback_at = body.index(fallback)
            self.assertLess(snapshot_return, fallback_at)

    def test_pre_and_post_physical_gates_are_parallel_without_crossing_detach(self) -> None:
        gate_at = SOURCE.index("\nmechanism_gate()\n")
        tail_at = SOURCE.index(
            '\nif [[ "${skip_pre_transport}" == 0 ]]; then\n', gate_at
        )
        tail = SOURCE[tail_at:]
        ordered = (
            "require_transport_contract",
            'parallel_detach_physical_gate "pre-detach" attached',
            'gz topic -t "${detach_topic}" -m gz.msgs.Empty',
            "require_transport_contract",
            'parallel_detach_physical_gate "post-detach" detached',
            "DOOR_SUPPORT_DETACH_8253=PASS",
        )
        cursor = 0
        for token in ordered:
            cursor = tail.index(token, cursor) + len(token)
        self.assertEqual(tail.count("parallel_detach_physical_gate \""), 2)
        self.assertEqual(tail.count("require_transport_contract"), 2)
        self.assertIn('if [[ "${skip_post_transport}" == 0 ]]; then', tail)
        self.assertIn('if [[ "${skip_pre_transport}" == 0 ]]; then', tail)
        self.assertIn("PRE_TRANSPORT=PRELOAD_ATTACH_CHECK_REUSED", tail)
        self.assertIn("POST_TRANSPORT=PRECHECK_REUSED", tail)
        triplet_start = SOURCE.index("\nmechanism_triplet_gate()\n")
        parallel_start = SOURCE.index("\nparallel_detach_physical_gate()\n")
        triplet = SOURCE[triplet_start:parallel_start]
        self.assertIn("for sample in 1 2 3", triplet)
        self.assertEqual(triplet.count('rows="$(mechanism_rows)"'), 1)
        self.assertIn("-n 3", triplet)

        parallel = SOURCE[parallel_start:tail_at]
        for token in (
            '[[ "$(support_state)" == "${expected_support}" ]]',
            'require_robot_close_endpoint "${phase}"',
            'mechanism_triplet_gate "${phase}"',
            'wait "${support_pid}"',
            'wait "${robot_pid}"',
            'wait "${mechanism_pid}"',
            "DOOR_SUPPORT_DETACH_PARALLEL_GATE=PASS",
        ):
            self.assertIn(token, parallel)

    def test_endpoint_latch_support_transport_and_thresholds_are_unchanged(self) -> None:
        endpoint = function_body("require_robot_close_endpoint", "mechanism_gate")
        rows_gate = function_body("mechanism_rows_gate", "mechanism_gate")
        gate = function_body("mechanism_gate", "mechanism_triplet_gate")
        triplet_start = SOURCE.index("\nmechanism_triplet_gate()\n")
        triplet_end = SOURCE.index("\nparallel_detach_physical_gate()\n", triplet_start)
        triplet = SOURCE[triplet_start:triplet_end]
        for token in (
            "d<=0.003",
            "count == 9",
            "endpoint cardinality is not 7+2",
        ):
            self.assertIn(token, endpoint)
        for token in (
            "door<=0.003490658503989",
            "h<=0.052359877559830",
            "dv<=0.002",
            "hv<=0.002",
            "timeout 5 gz topic -e -t /incubator/latch/state -n 1",
            "latch is not unlocked",
        ):
            if token.startswith(("door<=", "h<=", "dv<=", "hv<=")):
                self.assertIn(token, rows_gate)
            else:
                self.assertIn(token, gate)
        self.assertIn("count==3", triplet)
        self.assertIn('$2!="unlocked"', triplet)
        for token in (
            "support state must have exactly one StringMsg publisher",
            "support attach must have exactly one Empty subscriber",
            "support detach must have exactly one Empty subscriber",
        ):
            self.assertIn(token, SOURCE)

    def test_transport_discovery_is_parallel_but_all_three_results_are_waited(self) -> None:
        body = function_body("require_transport_contract", "support_state")
        self.assertEqual(body.count("timeout 5 gz topic -i -t"), 3)
        self.assertEqual(body.count("  wait \"${"), 3)
        for token in (
            'state_pid=$!',
            'attach_pid=$!',
            'detach_pid=$!',
            'wait "${state_pid}"',
            'wait "${attach_pid}"',
            'wait "${detach_pid}"',
            '(( discovery_failed == 0 ))',
        ):
            self.assertIn(token, body)

    def test_helper_still_contains_no_motion_reset_or_release_command(self) -> None:
        for forbidden in (
            "ros2 topic pub",
            "ros2 action send_goal",
            "gz model -m",
            "PoseCmd",
            "JointPositionReset",
            "right_gripper_controller/joint_trajectory -m",
            "reset --",
        ):
            self.assertNotIn(forbidden, SOURCE)
        self.assertEqual(
            SOURCE.count('gz topic -t "${detach_topic}" -m gz.msgs.Empty'), 1
        )


if __name__ == "__main__":
    unittest.main()
