import json
from pathlib import Path

from h1_2_navigation import v12_policy_export
from h1_2_navigation.locomotion_contract import file_identity
from h1_2_navigation.v12_policy import V12Policy
import numpy as np
import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
POLICY_DIR = PACKAGE_ROOT / 'policy'


def test_numpy_policy_matches_frozen_golden_corpus():
    policy = V12Policy.load(POLICY_DIR / 'v12_model_17250_policy.npz')
    with np.load(
            POLICY_DIR / 'v12_model_17250_golden.npz',
            allow_pickle=False) as golden:
        actual = policy.infer(golden['observations'])
        expected = golden['actions']

    np.testing.assert_allclose(actual, expected, rtol=5.0e-5, atol=5.0e-5)


def test_policy_manifest_binds_every_exported_file_and_exporter():
    manifest_path = POLICY_DIR / 'v12_model_17250_manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))

    assert manifest['policy'] == file_identity(
        POLICY_DIR / 'v12_model_17250_policy.npz')
    assert manifest['torchscript'] == file_identity(
        POLICY_DIR / 'v12_model_17250_policy.pt')
    assert manifest['golden'] == file_identity(
        POLICY_DIR / 'v12_model_17250_golden.npz')
    assert manifest['exporter_sha256'] == file_identity(
        Path(v12_policy_export.__file__))['sha256']


def test_policy_preserves_exact_zero_branch_boundary():
    policy = V12Policy.load(POLICY_DIR / 'v12_model_17250_policy.npz')
    observation = np.zeros((2, 78), dtype=np.float32)
    observation[1, 42] = np.nextafter(
        np.float32(0.0), np.float32(1.0))

    actions = policy.infer(observation)

    assert not np.array_equal(actions[0], actions[1])


def test_policy_rejects_invalid_observation():
    policy = V12Policy.load(POLICY_DIR / 'v12_model_17250_policy.npz')

    with pytest.raises(ValueError, match='shape'):
        policy.infer(np.zeros(77))
    observation = np.zeros(78)
    observation[4] = np.nan
    with pytest.raises(ValueError, match='non-finite'):
        policy.infer(observation)
