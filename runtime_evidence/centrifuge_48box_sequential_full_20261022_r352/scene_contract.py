import ast
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as element_tree


SCOPE = 'two_box_load_close_spin_open_retrieve'
KNOWN_V12_FAILURES = frozenset(('test_v12_checkpoint_identity_matches_contract',
                              'test_asset_audit_confirms_matches_and_blocks_unready_transfer'))
ALLOWED_NAVIGATION_EXECUTABLES = frozenset(('mid360_square_localizer',))
FORBIDDEN_TOKENS = ('locomotion', 'v12', 'unitree', 'velocity_gate', 'nav2', 'navigate_to_pose')


def verify_fixed_pelvis(contents):
    robot = element_tree.fromstring(contents)
    joints = [joint for joint in robot.findall('joint')
              if joint.find('parent') is not None and joint.find('parent').get('link') == 'world']
    if len(joints) != 1 or joints[0].get('type') != 'fixed' or joints[0].find('child').get('link') != 'pelvis':
        raise ValueError('Scene contract requires the existing fixed world-to-pelvis joint')
    links = {link.get('name') for link in robot.findall('link')}
    if not {'pelvis', 'left_wrist_yaw_link', 'right_wrist_yaw_link'} <= links:
        raise ValueError('Incomplete robot')


def verify_declared_nodes(contents):
    nodes = []
    for call in ast.walk(ast.parse(contents)):
        if not isinstance(call, ast.Call) or not isinstance(call.func, ast.Name) or call.func.id != 'Node':
            continue
        values = {keyword.arg:keyword.value for keyword in call.keywords}
        row = {}
        for name in ('package', 'executable'):
            value = values.get(name)
            if not isinstance(value, ast.Constant) or not isinstance(value.value, str):
                raise ValueError('Dynamic node identity requires a new dependency review')
            row[name] = value.value
        if any(token in (row['package'] + ':' + row['executable']).lower() for token in FORBIDDEN_TOKENS):
            raise ValueError('Walking or policy execution is outside this scene contract')
        if row['package'] == 'h1_2_navigation' and row['executable'] not in ALLOWED_NAVIGATION_EXECUTABLES:
            raise ValueError('Unreviewed navigation executable')
        nodes.append(row)
    return nodes


def classify_regression(contents):
    cases = element_tree.fromstring(contents).findall('.//testcase')
    failed = [case.get('name') for case in cases if case.find('failure') is not None]
    skipped = sum(case.find('skipped') is not None for case in cases)
    errors = sum(case.find('error') is not None for case in cases)
    if len(cases) != 113 or skipped or errors or not set(failed) <= KNOWN_V12_FAILURES or len(set(failed)) != len(failed):
        raise ValueError('Unexpected regression result; no scene launch permitted')
    for case in cases:
        failure = case.find('failure')
        if failure is not None:
            details = ''.join(failure.itertext())
            if 'FileNotFoundError' not in details or 'IsaacLab-3.0-beta2' not in details or 'model_17250.pt' not in details:
                raise ValueError('A known test failed for an unreviewed reason')
    return {'cases':len(cases), 'passed':len(cases)-len(failed), 'failed':failed,
            'skipped':skipped, 'errors':errors, 'original_regression_passed':not failed,
            'scene_dependencies_passed':True, 'unrelated_missing_sources_preserved':True}


def verify_scope(scope, walking_enabled, reconstructed_checkpoint):
    if scope != SCOPE or walking_enabled or reconstructed_checkpoint:
        raise ValueError('Only the independently reviewed fixed-pelvis hand cycle is authorized')


def verify_live_processes(commands):
    for command in commands:
        if any(token in command.lower() for token in FORBIDDEN_TOKENS):
            raise ValueError('Unexpected policy/navigation process in scene launch ancestry')


def verify_ctest_result(contents):
    summary = re.search(r'(?:^|\n)\s*100% tests passed, 0 tests failed out of ([1-9][0-9]*)', contents)
    if summary is None or re.search(r'\*\*\*(?:Failed|Not Run|Skipped)|Errors while running CTest', contents):
        raise ValueError('C++/package tests did not all pass')
    return int(summary.group(1))


def read_contract(root):
    contract = json.loads((Path(root) / 'scene_dependency_audit_revision2.json').read_text())
    if contract.get('status') != 'PASS':
        raise ValueError('Scene dependency audit has not passed')
    verify_scope(contract['scope'], contract['walking_enabled'], contract['reconstructed_checkpoint'])
    if not contract.get('full_hand_dependency_reviewed'):
        raise ValueError('Startup-only evidence cannot authorize the full hand cycle')
    for resource in contract['selected_resources']:
        path = Path(resource['path'])
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != resource['sha256']:
            raise ValueError('Selected hand dependency changed: ' + str(path))
    return contract


def verify_hand_imports(contents):
    for item in ast.walk(ast.parse(contents)):
        names = []
        if isinstance(item, ast.Import):
            names = [alias.name for alias in item.names]
        elif isinstance(item, ast.ImportFrom):
            names = [item.module or '']
        if any(token in name.lower() for name in names for token in FORBIDDEN_TOKENS):
            raise ValueError('Policy or navigation action import in fixed-pelvis hand cycle')
