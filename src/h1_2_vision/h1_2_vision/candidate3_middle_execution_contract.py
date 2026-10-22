"""Fail-closed execution admission for middle-v1 plan-only results."""

from __future__ import annotations

from collections.abc import Mapping

from .candidate3_middle_contract import (
    canonical_hash,
    CERTIFICATE_SCHEMA,
    make_certificate,
    MiddleContractError,
    SEAM_STATE_HASH,
    SOURCE_ASSET_HASHES,
    TOPOLOGY,
    validate_exact_audit_evidence,
    validate_middle_plan,
    WORKER_VERSION,
)


RESULT_SCHEMA = 'h1_candidate3_middle_result/v1'


class MiddleExecutionContractError(ValueError):
    """Raised when a result is not admissible for any future executor."""


def _require(condition: bool, status: str) -> None:
    if not condition:
        raise MiddleExecutionContractError(status)


def validate_result(result: Mapping) -> Mapping:
    """Return the plan only when every middle-v1 certificate gate passes."""
    _require(isinstance(result, Mapping), 'RESULT_TYPE')
    _require(result.get('schema') == RESULT_SCHEMA, 'RESULT_SCHEMA')
    plan = result.get('plan')
    evidence = result.get('audit_evidence')
    certificate = result.get('certificate')
    _require(isinstance(plan, Mapping), 'RESULT_PLAN')
    _require(isinstance(evidence, Mapping), 'RESULT_AUDIT_EVIDENCE')
    _require(isinstance(certificate, Mapping), 'RESULT_CERTIFICATE')
    try:
        validate_middle_plan(plan)
        validate_exact_audit_evidence(evidence, plan)
    except MiddleContractError as error:
        raise MiddleExecutionContractError(str(error)) from error
    plan_hash = canonical_hash(plan)
    evidence_hash = canonical_hash(evidence)
    _require(result.get('plan_hash') == plan_hash, 'RESULT_PLAN_HASH')
    _require(result.get('audit_evidence_hash') == evidence_hash,
             'RESULT_AUDIT_HASH')
    _require(result.get('certificate_hash') == canonical_hash(certificate),
             'RESULT_CERTIFICATE_HASH')
    expected = make_certificate(plan, evidence)
    _require(certificate == expected, 'CERTIFICATE_CONTENT')
    _require(certificate.get('schema') == CERTIFICATE_SCHEMA,
             'CERTIFICATE_SCHEMA')
    _require(certificate.get('worker_version') == WORKER_VERSION,
             'CERTIFICATE_VERSION')
    _require(certificate.get('topology') == TOPOLOGY,
             'CERTIFICATE_TOPOLOGY')
    _require(certificate.get('source_asset_hashes') == SOURCE_ASSET_HASHES,
             'CERTIFICATE_SOURCE_ASSETS')
    _require(certificate.get('seam_state_hash') == SEAM_STATE_HASH,
             'CERTIFICATE_SEAM_HASH')
    _require(certificate.get('left_handle_stage_count') == 0,
             'CERTIFICATE_LEFT_HANDLE_STAGE')
    _require(certificate.get('torso_controller_authorized') is False,
             'CERTIFICATE_TORSO_CONTROLLER')
    _require(certificate.get('controller_commands_issued') == 0,
             'CERTIFICATE_CONTROLLER_COMMANDS')
    _require(certificate.get('valid') is True and
             certificate.get('status') == 'PASS', 'CERTIFICATE_INVALID')
    return plan
