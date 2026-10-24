VALIDATION_FILES = frozenset(('minimum_turn_tests_final.log', 'tests_revision3.log', 'front_camera_tests_final.log', 'optional_micro_tests.log', 'dense_endpoint_tests.log', 'lift_provider_tests.log', 'package_tests.log', 'navigation_tests.log',
    'loading_regression_tests_final.log', 'build.log', 'prepare.log', 'buttons_audit.log',
    'conda_six_before.txt', 'conda_six_after.txt', 'navigation_tests.xml', 'ctest.log',
    'resize_preflight.log', 'resize_decode.log', 'loaded_dispatch_tests.log',
    'complete_profile_dispatch_tests.log', 'private_lid_rate_tests.log', 'button_ramp_tests.log',
    'retrieval_rate_restore_tests.log', 'schedule_gripper_tests.log', 'native_gripper_feedback_tests.log',
    'dense_shortcut_tests.log', 'dense_fixture_scope_tests.log', 'gentle_setdown_tests.log',
    'initial_approach_tests.log', 'startup_ramp_tests.log', 'post_storage_tests.log', 'retrieval_grip_tests.log',
    'pickup_approach_tests.log', 'post_retreat_gripper_tests.log', 'coarse_ramp_tests.log', 'empty_protocol_tests.log'))


VALIDATION_FILES = VALIDATION_FILES | frozenset(('elbow_empty_approach_tests.log',))
VALIDATION_FILES = VALIDATION_FILES | frozenset(('elbow_transport_rate_tests.log',))
VALIDATION_FILES = VALIDATION_FILES | frozenset(('startup_review_hold_tests.log',))
VALIDATION_FILES = VALIDATION_FILES | frozenset(('startup_review_freeze_tests.log',))


def is_identity_member(path):
    selected = path.suffix in ('.py', '.sh', '.md', '.json') or path.name in VALIDATION_FILES
    return selected and path.name not in ('identity.json', 'SHA256SUMS', 'REPORT.md')

