"""Offline-only builder for candidate-3 middle-v1 audit inputs/results."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys

from .candidate3_middle_contract import (
    ASSET_SPEC_BY_NAME,
    build_middle_plan,
    canonical_hash,
    CERTIFICATE_SCHEMA,
    default_asset_paths,
    load_middle_assets,
    make_certificate,
    PHASE_SPECS,
    SEAM_STATE_HASH,
    SOURCE_ASSET_HASHES,
    TOPOLOGY,
    WORKER_VERSION,
)
from .candidate3_middle_execution_contract import RESULT_SCHEMA


PHASE_TIME_SCALES = {
    'door_pre_support':
        '0:0.25,1:0.08,2:0.3,3:0.12,4:0.35,5:0.05',
    'door_open_supported':
        '6:0.1,7:0.12,8:0.08,9:0.08,10:0.08,11:0.08,12:0.08',
    'cube_contact': '0:0.36,1:0.08',
    'cube_carry_place': '2:0.36,3:0.36',
    'cube_release': '4:0.08',
    'cube_retreat': '5:0.36',
    'door_close_supported':
        '0:0.08,1:0.08,2:0.08,3:0.08,4:0.08,5:0.12,6:0.1',
    'right_release': '7:0.08',
    'right_retreat': '8:0.341,9:0.105,10:0.285,11:0.075,12:0.235',
}
RETIME_PATTERN = re.compile(
    r'^RETIME_CHECK=PASS stage=(?P<stage>door|cube) '
    r'range=\[(?P<start>[0-9]+),(?P<end>[0-9]+)\) '
    r'messages=(?P<messages>[0-9]+) overrides=(?P<overrides>[0-9]+) '
    r'original_duration_seconds=(?P<original>[0-9]+\.[0-9]{9}) '
    r'retimed_duration_seconds=(?P<retimed>[0-9]+\.[0-9]{9}) '
    r'original_critical_path_seconds=(?P<original_critical>[0-9]+\.[0-9]{9}) '
    r'retimed_critical_path_seconds=(?P<retimed_critical>[0-9]+\.[0-9]{9}) '
    r'sha256=(?P<sha256>[0-9a-f]{64})$')


class MiddleWorkerError(RuntimeError):
    """Raised for an offline worker failure."""


def _retime_audits(
        asset_tool: Path, paths: dict[str, Path]) -> list[dict]:
    """Run only the asset tool's non-publishing ``retime-check`` command."""
    if not asset_tool.is_file() or asset_tool.is_symlink():
        raise MiddleWorkerError('ASSET_TOOL_PATH')
    audits = []
    for phase in PHASE_SPECS:
        spec = ASSET_SPEC_BY_NAME[phase.asset]
        command = [
            str(asset_tool), 'retime-check', str(paths[phase.asset]),
            '--expected-stage', spec.stage,
            '--start-index', str(phase.start), '--end-index', str(phase.end),
            '--message-time-scales', PHASE_TIME_SCALES[phase.name],
        ]
        completed = subprocess.run(
            command, check=False, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, timeout=30.0)
        line = completed.stdout.strip()
        match = RETIME_PATTERN.fullmatch(line)
        if completed.returncode != 0 or match is None:
            raise MiddleWorkerError(f'RETIME_CHECK:{phase.name}:{line}')
        values = match.groupdict()
        if (int(values['start']) != phase.start or
                int(values['end']) != phase.end or
                int(values['messages']) != phase.end - phase.start or
                values['sha256'] != spec.payload_sha256):
            raise MiddleWorkerError(f'RETIME_IDENTITY:{phase.name}')
        audits.append({
            'phase': phase.name,
            'message_range': [phase.start, phase.end],
            'message_time_scales': PHASE_TIME_SCALES[phase.name],
            'retimed_duration_s': float(values['retimed']),
            'retimed_critical_path_s': float(values['retimed_critical']),
            'asset_payload_sha256': values['sha256'],
            'status': 'PASS',
        })
    return audits


def _pending_certificate(plan_hash: str) -> dict:
    return {
        'schema': CERTIFICATE_SCHEMA,
        'worker_version': WORKER_VERSION,
        'topology': TOPOLOGY,
        'plan_hash': plan_hash,
        'audit_evidence_hash': '',
        'source_asset_hashes': dict(SOURCE_ASSET_HASHES),
        'seam_state_hash': SEAM_STATE_HASH,
        'left_handle_stage_count': 0,
        'torso_controller_authorized': False,
        'controller_commands_issued': 0,
        'valid': False,
        'status': 'EXACT_GEOMETRY_AUDIT_REQUIRED',
    }


def build_result(
        workspace: Path, scene_epoch_id: int, raw_scene_hash: str,
        physical_scene_hash: str, asset_tool: Path,
        audit_evidence: dict | None = None) -> dict:
    """Build a deterministic pending result or a fully evidenced result."""
    paths = default_asset_paths(workspace)
    assets = load_middle_assets(paths)
    timing = _retime_audits(asset_tool, paths)
    plan = build_middle_plan(
        assets, scene_epoch_id=scene_epoch_id,
        raw_scene_hash=raw_scene_hash,
        physical_scene_hash=physical_scene_hash)
    plan['retime_audits'] = timing
    plan['planned_action_duration_s'] = sum(
        value['retimed_duration_s'] for value in timing)
    plan['timing_audit_status'] = 'PASS'
    plan_hash = canonical_hash(plan)
    if audit_evidence is None:
        certificate = _pending_certificate(plan_hash)
        evidence_hash = ''
    else:
        certificate = make_certificate(plan, audit_evidence)
        evidence_hash = canonical_hash(audit_evidence)
    return {
        'schema': RESULT_SCHEMA,
        'plan': plan,
        'plan_hash': plan_hash,
        'audit_evidence': audit_evidence,
        'audit_evidence_hash': evidence_hash,
        'certificate': certificate,
        'certificate_hash': canonical_hash(certificate),
    }


def _atomic_json(path: Path, value) -> None:
    path = path.resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f'.{path.name}.{os.getpid()}.tmp')
    temporary.write_text(
        json.dumps(value, sort_keys=True, indent=2, allow_nan=False) + '\n',
        encoding='utf-8')
    os.replace(temporary, path)


def parse_arguments(argv=None):
    """Parse an explicitly offline-only invocation."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--scene-epoch-id', type=int, required=True)
    parser.add_argument('--raw-scene-hash', required=True)
    parser.add_argument('--physical-scene-hash', required=True)
    parser.add_argument('--asset-tool', type=Path, required=True)
    parser.add_argument('--audit-evidence', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    return parser.parse_args(argv)


def main(argv=None) -> int:
    """Create an audit result without importing ROS or opening controllers."""
    try:
        args = parse_arguments(argv)
        evidence = None
        if args.audit_evidence is not None:
            evidence = json.loads(
                args.audit_evidence.read_text(encoding='utf-8'))
        result = build_result(
            args.workspace, args.scene_epoch_id, args.raw_scene_hash,
            args.physical_scene_hash, args.asset_tool, evidence)
        _atomic_json(args.output, result)
        print(json.dumps({
            'event': 'CANDIDATE3_MIDDLE_PLAN_ONLY',
            'plan_hash': result['plan_hash'],
            'certificate_valid': result['certificate']['valid'],
            'status': result['certificate']['status'],
            'controller_commands_issued': 0,
        }, sort_keys=True))
        return 0 if result['certificate']['valid'] else 2
    except (
            MiddleWorkerError, OSError, ValueError,
            subprocess.SubprocessError) as error:
        print(f'CANDIDATE3_MIDDLE_PLAN_ONLY=ERROR:{error}', file=sys.stderr)
        return 3


if __name__ == '__main__':
    raise SystemExit(main())
