#!/usr/bin/env python3
"""Pure-source fail-closed contract for h1_door_support_8251.sh."""

from pathlib import Path
import re
import subprocess
import unittest


HELPER = (
    Path(__file__).resolve().parents[1] / "scripts" / "h1_door_support_8251.sh"
)
SOURCE = HELPER.read_text(encoding="utf-8")


def function_body(name: str, next_name: str | None = None) -> str:
    start = SOURCE.index(f"\n{name}()\n")
    if next_name is None:
        end = SOURCE.index('\nif [[ "${operation}" == "preload" ]]', start)
    else:
        end = SOURCE.index(f"\n{next_name}()\n", start)
    return SOURCE[start:end]


class DoorSupportSourceContract(unittest.TestCase):
    def test_bash_parser_accepts_helper(self) -> None:
        subprocess.run(["bash", "-n", str(HELPER)], check=True)

    def test_caps_and_request_fields_cannot_accept_markup(self) -> None:
        self.assertIn(
            '[[ "${value}" =~ ^(0|[1-9][0-9]*)(\\.[0-9]+)?$ ]]', SOURCE
        )
        self.assertIn('is_bounded_positive_decimal "${max_force}" 250', SOURCE)
        self.assertIn('is_bounded_positive_decimal "${max_torque}" 20', SOURCE)
        self.assertIn(
            '[[ "${plugin_so}" =~ ^/[-A-Za-z0-9_./+]+$ ]]', SOURCE
        )
        self.assertLess(
            SOURCE.index('is_bounded_positive_decimal "${max_force}" 250'),
            SOURCE.index('request="entity {id:'),
        )

    def test_split_is_exclusive_and_tail_is_seven_right_arm_state_segments(self) -> None:
        for token in (
            'all($messages[];',
            '.joint_names == $right_arm_joints',
            '.joint_names == $right_finger_joints',
            '$messages[$attach - 1].topic ==',
            '($messages[$attach:] | length) == 7',
            'all($messages[$attach:][];',
        ):
            self.assertIn(token, SOURCE)
        self.assertIn(
            '.payload.messages[0:$boundary][] | select(.topic==$topic)', SOURCE
        )

    def test_endpoint_requires_exact_right_joint_sets_and_clamp_values(self) -> None:
        for joint in (
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint",
            "right_dex1_finger_1_joint",
            "right_dex1_finger_2_joint",
        ):
            self.assertIn(joint, SOURCE)
        self.assertIn("nominal=0.01131", SOURCE)
        self.assertIn("nominal=0.01235", SOURCE)
        self.assertIn("actual_error<=0.0015 && expected_error<=0.0015", SOURCE)
        self.assertIn('is_finite_decimal_text "${expected}"', SOURCE)
        self.assertIn('is_finite_decimal_text "${actual}"', SOURCE)

    def test_transport_contract_checks_direction_type_and_exact_count(self) -> None:
        for token in (
            "start_state_listener before_add",
            "wait_for_listener_discovery resident",
            "require_resident_transport_contract",
            '"${listener_mode}" == "without_listener"',
            'No publishers on topic [${attach_topic}]',
            'No publishers on topic [${detach_topic}]',
            "google.protobuf.Message",
            "gz.msgs.StringMsg",
            "gz.msgs.Empty",
            "Service providers [Address, Request Message Type, Response Message Type]:",
            'providers==1 && !bad',
        ):
            self.assertIn(token, SOURCE)
        request_at = SOURCE.index('request="entity {id:')
        self.assertLess(SOURCE.index("start_state_listener before_add"), request_at)
        self.assertGreater(
            SOURCE.index("wait_for_listener_discovery resident", request_at), request_at
        )

    def test_request_targets_only_the_existing_door_link_with_bounded_wrench(self) -> None:
        match = re.search(r'^\s*request="(.*)"$', SOURCE, flags=re.MULTILINE)
        self.assertIsNotNone(match)
        request = match.group(1)
        for token in (
            'entity {id: ${incubator_id}, name: \\"incubator\\", type: MODEL}',
            '<parent_scope>h1_2::right_wrist_yaw_link</parent_scope>',
            '<child_scope>incubator::door_link</child_scope>',
            '<gravity_compensation>false</gravity_compensation>',
            '<max_force>${max_force}</max_force>',
            '<max_torque>${max_torque}</max_torque>',
            '<initially_attached>false</initially_attached>',
        ):
            self.assertIn(token, request)
        self.assertEqual(SOURCE.count("<child_scope>incubator::door_link</child_scope>"), 1)
        self.assertIn('--req "${request}"', SOURCE)

    def test_preload_and_attach_are_separate_fail_closed_operations(self) -> None:
        preload = function_body("run_preload", "run_attach")
        attach = function_body("run_attach")
        self.assertIn('"${operation}" == "preload" || "${operation}" == "attach" ||', SOURCE)
        self.assertIn("usage: h1_door_support_8251.sh preload|attach|recover_attach", SOURCE)
        for token in (
            "require_empty_transport_contract",
            "require_add_service_contract",
            "start_state_listener before_add",
            "<initially_attached>false</initially_attached>",
            'gz service -s "${add_service}"',
            "require_fresh_state detached",
            "require_resident_transport_contract with_listener",
            "create_ownership_evidence",
            "DOOR_SUPPORT_PRELOAD=PASS child=incubator::door_link",
        ):
            self.assertIn(token, preload)
        for token in (
            "require_ownership_evidence",
            "require_resident_transport_contract without_listener",
            "start_mechanism_pulse_monitor",
            "require_unlocked_latch",
            'asset_endpoint_gate "/right_arm_controller/joint_trajectory" 7',
            'asset_endpoint_gate "/right_gripper_controller/joint_trajectory" 2',
            "start_state_listener resident",
            "require_fresh_state detached",
            'gz topic -t "${attach_topic}" -m gz.msgs.Empty',
            "require_fresh_state attached",
            "require_resident_transport_contract with_listener",
            "post-attach right-arm endpoint",
            "post-attach right-finger endpoint",
            "DOOR_SUPPORT_HOOK=PASS operation=attach",
        ):
            self.assertIn(token, attach)
        self.assertNotIn('gz service -s "${add_service}"', attach)
        self.assertNotIn("<initially_attached>", attach)
        self.assertEqual(SOURCE.count("<initially_attached>false</initially_attached>"), 1)

    def test_fast_resident_mode_is_opt_in_and_attach_only(self) -> None:
        self.assertIn(
            'readonly fast_resident="${H1_DOOR_SUPPORT_FAST_RESIDENT:-0}"',
            SOURCE,
        )
        self.assertIn(
            '[[ "${fast_resident}" == "0" || "${fast_resident}" == "1" ]]',
            SOURCE,
        )
        self.assertIn(
            '[[ "${fast_resident}" == "0" || "${operation}" == "attach" ||',
            SOURCE,
        )
        self.assertIn(
            "H1_DOOR_SUPPORT_FAST_RESIDENT=1 is valid only for attach after preload",
            SOURCE,
        )
        self.assertIn("recovery ownership parent is still active", SOURCE)
        self.assertIn("recovery ownership identity reconstruction failed", SOURCE)

    def test_fast_resident_skips_only_preload_proofs_and_discovery(self) -> None:
        attach = function_body("run_attach")
        for retained in (
            "require_ownership_evidence",
            "start_mechanism_pulse_monitor",
            "require_unlocked_latch",
            'asset_endpoint_gate "/right_arm_controller/joint_trajectory" 7',
            'asset_endpoint_gate "/right_gripper_controller/joint_trajectory" 2',
            "require_fresh_state detached",
            'gz topic -t "${attach_topic}" -m gz.msgs.Empty',
            "require_fresh_state attached",
            "mechanism_pulse_gate",
            "post-attach right-arm endpoint",
            "post-attach right-finger endpoint",
        ):
            self.assertIn(retained, attach)
        self.assertIn('if [[ "${fast_resident}" == "0" ]]; then', attach)
        self.assertIn("require_resident_transport_contract without_listener", attach)
        self.assertIn("require_resident_transport_contract with_listener", attach)
        self.assertIn("start_state_listener fast_resident", attach)
        self.assertIn("DOOR_SUPPORT_FAST_RESIDENT=PASS", attach)

        listener = function_body("start_state_listener", "require_fresh_state")
        self.assertIn('"${discovery_mode}" != "fast_resident"', listener)
        self.assertIn('wait_for_listener_discovery "${discovery_mode}"', listener)

        static_start = SOURCE.index('if [[ "${fast_resident}" == "0" ]]; then')
        static_end = SOURCE.index("\ndoor_asset_real=", static_start)
        static_checks = SOURCE[static_start:static_end]
        self.assertIn('"${asset_tool}" inspect "${door_asset}"', static_checks)
        self.assertIn("all($messages[];", static_checks)

    def test_continuous_pulse_monitor_spans_attach_publication_before_pass(self) -> None:
        attach = function_body("run_attach")
        start_call = attach.index("\n  start_mechanism_pulse_monitor\n")
        publish_at = attach.index('gz topic -t "${attach_topic}"')
        stop_call = attach.index("\n  stop_mechanism_pulse_monitor\n", publish_at)
        pulse_call = attach.index("\n  mechanism_pulse_gate\n", stop_call)
        pass_at = attach.index("DOOR_SUPPORT_HOOK=PASS")
        self.assertLess(start_call, publish_at)
        self.assertLess(publish_at, stop_call)
        self.assertLess(stop_call, pulse_call)
        self.assertLess(pulse_call, pass_at)
        self.assertIn(
            "PYTHONUNBUFFERED=1 timeout 45 ros2 topic echo /incubator/joint_states",
            SOURCE,
        )
        self.assertIn("pre_attach_mechanism_sample_count", SOURCE)
        self.assertIn("refresh_mechanism_monitor_baseline", SOURCE)
        self.assertIn(
            "sample_count >= pre_attach_mechanism_sample_count + 3", SOURCE
        )
        self.assertIn("door>0.003490658503989", SOURCE)
        self.assertIn("door_velocity>0.002", SOURCE)

    def test_snapshot_tool_is_preferred_with_read_only_ros_fallbacks(self) -> None:
        self.assertIn('readonly ros_snapshot_tool="${H1_FULL_ROS_SNAPSHOT_TOOL:-}"', SOURCE)
        self.assertIn('"${ros_snapshot_tool}" == /* && -f "${ros_snapshot_tool}"', SOURCE)
        self.assertIn('-x "${ros_snapshot_tool}"', SOURCE)
        self.assertIn('timeout 5 "${ros_snapshot_tool}" robot --timeout 3', SOURCE)
        self.assertIn('timeout 5 "${ros_snapshot_tool}" incubator --timeout 3', SOURCE)
        self.assertIn("ros2 topic echo --once /joint_states", SOURCE)
        self.assertIn("ros2 topic echo --once /incubator/joint_states", SOURCE)

    def test_helper_contains_no_direct_motion_or_release_command(self) -> None:
        for forbidden in (
            "ros2 topic pub",
            "ros2 action send_goal",
            "gz model -m",
            "PoseCmd",
            "JointPositionReset",
            'gz topic -t "${detach_topic}"',
        ):
            self.assertNotIn(forbidden, SOURCE)


if __name__ == "__main__":
    unittest.main()
