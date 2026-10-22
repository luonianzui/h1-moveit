"""
ROS-free admission CLI for a sealed candidate-3 middle result.

The only external semantic authority called by this module is
``validate_result``.  File identity, owner/mode/link count, strict JSON, and
the explicit file hash are checked first.  The returned plan must then expose
the frozen eleven-phase topology and canonical E0 held seam.  This module has
no ROS import, publisher, or execution code.
"""

from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import math
import os
from pathlib import Path
import re
import stat
import sys
from typing import Any, Dict, Mapping, Sequence, Tuple

from .candidate3_middle_execution_contract_828 import validate_result


SHA256_RE = re.compile(r'[0-9a-f]{64}\Z')
MAX_RESULT_BYTES = 1024 * 1024 * 1024
PHASE_NAMES = (
    'door_pre_support',
    'door_open_supported',
    'door60_seam_to_cube',
    'left_cube_ready',
    'cube_contact',
    'cube_carry_place',
    'cube_release',
    'cube_retreat',
    'left_cube_park',
    'door60_seam_to_hinge',
    'door_close_supported',
)
RELEASE_OWNER = 'suffix_after_q90_to_q0'
E0_TORSO_RAD = math.radians(-2.7349115403864994)
E0_RIGHT_ARM_RAD = (
    -0.41373146194762,
    0.23755434038212944,
    1.1008080707922079,
    0.8747783439736734,
    -0.3046852067271319,
    0.4094159191441438,
    -0.9999635680199944,
)
E0_RIGHT_FINGERS = {
    'right_dex1_finger_1_joint': 0.0108,
    'right_dex1_finger_2_joint': 0.007,
}
FORBIDDEN_MIDDLE_TEXT = ('right_release', 'right_retreat')


class MiddleAdmissionError(RuntimeError):
    """A sealed-file or semantic admission failure."""


def _require(condition: bool, status: str) -> None:
    if not condition:
        raise MiddleAdmissionError(status)


def _reject_duplicate_keys(
        pairs: Sequence[Tuple[str, Any]]) -> Dict[str, Any]:
    output: Dict[str, Any] = {}
    for key, value in pairs:
        if key in output:
            raise MiddleAdmissionError(f'RESULT_DUPLICATE_KEY:{key}')
        output[key] = value
    return output


def _reject_constant(value: str) -> None:
    raise MiddleAdmissionError(f'RESULT_NONFINITE:{value}')


def _identity(metadata: os.stat_result) -> Tuple[int, int, int, int, int]:
    return (
        metadata.st_dev,
        metadata.st_ino,
        metadata.st_size,
        metadata.st_mtime_ns,
        metadata.st_ctime_ns,
    )


def _read_result(path: Path, expected_sha256: str) -> Mapping[str, Any]:
    _require(
        type(expected_sha256) is str
        and SHA256_RE.fullmatch(expected_sha256) is not None,
        'RESULT_EXPECTED_SHA256',
    )
    path = Path(path)
    _require(path.is_absolute(), 'RESULT_PATH_NOT_ABSOLUTE')
    try:
        metadata = path.lstat()
    except OSError as error:
        raise MiddleAdmissionError(f'RESULT_LSTAT:{error}') from error
    _require(stat.S_ISREG(metadata.st_mode), 'RESULT_NOT_REGULAR')
    _require(not path.is_symlink(), 'RESULT_SYMLINK')
    _require(metadata.st_uid == os.geteuid(), 'RESULT_OWNER')
    _require(stat.S_IMODE(metadata.st_mode) == 0o400, 'RESULT_MODE')
    _require(metadata.st_nlink == 1, 'RESULT_NLINK')
    _require(0 < metadata.st_size <= MAX_RESULT_BYTES, 'RESULT_SIZE')
    flags = (
        os.O_RDONLY
        | getattr(os, 'O_CLOEXEC', 0)
        | getattr(os, 'O_NOFOLLOW', 0)
    )
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        raise MiddleAdmissionError(f'RESULT_OPEN:{error}') from error
    try:
        before = os.fstat(descriptor)
        _require(_identity(before) == _identity(metadata),
                 'RESULT_RACE_BEFORE')
        remaining = before.st_size
        digest = hashlib.sha256()
        chunks = []
        while remaining:
            block = os.read(descriptor, min(1024 * 1024, remaining))
            _require(bool(block), 'RESULT_SHORT_READ')
            chunks.append(block)
            digest.update(block)
            remaining -= len(block)
        _require(not os.read(descriptor, 1), 'RESULT_GREW')
        after = os.fstat(descriptor)
        _require(_identity(before) == _identity(after), 'RESULT_RACE_AFTER')
    finally:
        os.close(descriptor)
    _require(
        hmac.compare_digest(digest.hexdigest(), expected_sha256),
        'RESULT_SHA256',
    )
    try:
        value = json.loads(
            b''.join(chunks).decode('utf-8'),
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_constant,
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise MiddleAdmissionError(f'RESULT_JSON:{error}') from error
    _require(type(value) is dict, 'RESULT_ROOT')
    return value


def _all_strings(value: Any):
    if type(value) is str:
        yield value
    elif type(value) is list:
        for item in value:
            yield from _all_strings(item)
    elif type(value) is dict:
        for key, item in value.items():
            yield key
            yield from _all_strings(item)


def validate_terminal_seam(plan: Mapping[str, Any]) -> None:
    """Require the frozen middle topology and canonical held E0 endpoint."""
    _require(type(plan) is dict, 'MIDDLE_PLAN_TYPE')
    _require(tuple(plan.get('phase_names', ())) == PHASE_NAMES,
             'MIDDLE_PHASE_NAMES')
    _require(plan.get('release_owner') == RELEASE_OWNER,
             'MIDDLE_RELEASE_OWNER')
    for key, item in plan.items():
        if key == 'terminal_seam':
            continue
        for text in (key, *_all_strings(item)):
            _require(
                not any(forbidden in text
                        for forbidden in FORBIDDEN_MIDDLE_TEXT),
                'MIDDLE_FORBIDDEN_RIGHT_RELEASE_RETREAT',
            )
    seam = plan.get('terminal_seam')
    _require(type(seam) is dict, 'MIDDLE_TERMINAL_SEAM')
    _require(seam.get('release_owner') == RELEASE_OWNER,
             'MIDDLE_TERMINAL_SEAM_RELEASE_OWNER')
    _require(seam.get('forbidden_middle_phases') ==
             ['right_release', 'right_retreat'],
             'MIDDLE_TERMINAL_SEAM_FORBIDDEN_PHASES')
    _require(seam.get('valid') is True, 'MIDDLE_TERMINAL_SEAM_VALID')
    state = seam.get('return_e0')
    _require(type(state) is dict, 'MIDDLE_TERMINAL_SEAM_RETURN_E0')
    torso = state.get('torso_yaw_rad')
    _require(type(torso) in (int, float) and type(torso) is not bool
             and math.isfinite(torso)
             and abs(float(torso) - E0_TORSO_RAD) <= 1e-12,
             'MIDDLE_TERMINAL_SEAM_TORSO')
    arm = state.get('right_arm_rad')
    _require(type(arm) is list and len(arm) == 7,
             'MIDDLE_TERMINAL_SEAM_ARM')
    for index, (actual, expected) in enumerate(zip(arm, E0_RIGHT_ARM_RAD)):
        _require(
            type(actual) in (int, float)
            and type(actual) is not bool
            and math.isfinite(actual)
            and abs(float(actual) - expected) <= 1e-12,
            f'MIDDLE_TERMINAL_SEAM_ARM:{index}',
        )
    fingers = state.get('right_fingers_m')
    _require(type(fingers) is dict
             and frozenset(fingers) == frozenset(E0_RIGHT_FINGERS),
             'MIDDLE_TERMINAL_SEAM_FINGERS')
    for name, expected in E0_RIGHT_FINGERS.items():
        actual = fingers[name]
        _require(
            type(actual) in (int, float)
            and type(actual) is not bool
            and math.isfinite(actual)
            and abs(float(actual) - expected) <= 1e-12,
            f'MIDDLE_TERMINAL_SEAM_FINGER:{name}',
        )


def admit_result(path: Path, expected_sha256: str) -> Mapping[str, Any]:
    """Return the admitted plan without importing ROS or issuing commands."""
    result = _read_result(path, expected_sha256)
    try:
        plan = validate_result(result)
    except Exception as error:
        raise MiddleAdmissionError(f'RESULT_CONTRACT:{error}') from error
    validate_terminal_seam(plan)
    return plan


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--result-sha256', required=True)
    parser.add_argument('--quiet', action='store_true')
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run the result-only admission CLI."""
    args = _parser().parse_args(argv)
    try:
        plan = admit_result(args.result, args.result_sha256)
        if not args.quiet:
            print(
                'CANDIDATE3_MIDDLE_ADMISSION=PASS '
                f"plan_schema={plan.get('schema', '<missing>')} "
                'terminal_seam=canonical_e0_q90_held '
                'controller_commands=0 ros_imports=0'
            )
        return 0
    except MiddleAdmissionError as error:
        print(f'CANDIDATE3_MIDDLE_ADMISSION=FAIL:{error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
