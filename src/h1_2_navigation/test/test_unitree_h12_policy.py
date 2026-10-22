import json
from pathlib import Path

import numpy as np
import pytest

from h1_2_navigation.unitree_h12_contract import file_identity
from h1_2_navigation.unitree_h12_policy import UnitreeH12Policy
from h1_2_navigation import unitree_h12_policy_export


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
POLICY_DIR = PACKAGE_ROOT / 'policy'


def _golden_actions(policy, observations):
    policy.reset()
    return np.asarray([
        policy.infer(observation) for observation in observations
    ], dtype=np.float32)


def test_numpy_lstm_matches_frozen_golden_sequence():
    policy = UnitreeH12Policy.load(
        POLICY_DIR / 'unitree_h12_motion_policy.npz')
    with np.load(
            POLICY_DIR / 'unitree_h12_motion_golden.npz',
            allow_pickle=False) as golden:
        actual = _golden_actions(policy, golden['observations'])
        expected = golden['actions']

    np.testing.assert_allclose(actual, expected, rtol=5.0e-5, atol=5.0e-5)


def test_recurrent_reset_reproduces_the_same_sequence():
    policy = UnitreeH12Policy.load(
        POLICY_DIR / 'unitree_h12_motion_policy.npz')
    with np.load(
            POLICY_DIR / 'unitree_h12_motion_golden.npz',
            allow_pickle=False) as golden:
        observations = golden['observations'][:12]
    first = _golden_actions(policy, observations)
    second = _golden_actions(policy, observations)

    np.testing.assert_array_equal(first, second)


def test_manifest_binds_policy_golden_source_and_exporter():
    manifest = json.loads((
        POLICY_DIR / 'unitree_h12_motion_manifest.json'
    ).read_text(encoding='utf-8'))

    assert manifest['policy'] == file_identity(
        POLICY_DIR / 'unitree_h12_motion_policy.npz')
    assert manifest['golden'] == file_identity(
        POLICY_DIR / 'unitree_h12_motion_golden.npz')
    assert manifest['source_policy'] == {
        'path': str(Path(
            '/home/liu/robot/gym_sim/unitree_rl_gym-main/deploy/'
            'pre_train/h1_2/motion.pt').resolve()),
        **file_identity(
            '/home/liu/robot/gym_sim/unitree_rl_gym-main/deploy/'
            'pre_train/h1_2/motion.pt'),
    }
    assert manifest['exporter_sha256'] == file_identity(
        Path(unitree_h12_policy_export.__file__))['sha256']
    assert manifest['numpy_maximum_absolute_error'] <= 5.0e-5


def test_policy_rejects_invalid_observation():
    policy = UnitreeH12Policy.load(
        POLICY_DIR / 'unitree_h12_motion_policy.npz')

    with pytest.raises(ValueError, match='shape'):
        policy.infer(np.zeros(46))
    observation = np.zeros(47)
    observation[3] = np.nan
    with pytest.raises(ValueError, match='non-finite'):
        policy.infer(observation)
