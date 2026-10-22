"""Pure NumPy inference for the frozen V12 locomotion actor."""

from pathlib import Path

import numpy as np


OBSERVATION_DIMENSION = 78
ACTION_DIMENSION = 12
COMMAND_SLICE = slice(42, 45)
CONTEXT_SLICE = slice(45, 57)
NORMALIZER_EPSILON = np.float32(1.0e-2)

CONTEXT_SCALE = np.asarray([
    2.0 / 3.0, 4.0 / 3.0, 2.0 / 3.0, 4.0 / 3.0,
    2.0 / 3.0, 2.0, 1.0, 1.0, 1.0, 1.0, 2.0, 2.0,
], dtype=np.float32)
CONTEXT_BIAS = np.asarray([
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -1.0, -1.0,
], dtype=np.float32)
CONTEXT_CLIP = np.float32(3.0)

EXPECTED_ARRAY_SHAPES = {
    'normalizer_mean': (1, OBSERVATION_DIMENSION),
    'normalizer_std': (1, OBSERVATION_DIMENSION),
    'motion_0_weight': (128, OBSERVATION_DIMENSION),
    'motion_0_bias': (128,),
    'motion_2_weight': (128, 128),
    'motion_2_bias': (128,),
    'motion_4_weight': (128, 128),
    'motion_4_bias': (128,),
    'motion_6_weight': (128, 128),
    'motion_6_bias': (128,),
    'motion_8_weight': (ACTION_DIMENSION, 128),
    'motion_8_bias': (ACTION_DIMENSION,),
    'stop_0_weight': (64, OBSERVATION_DIMENSION),
    'stop_0_bias': (64,),
    'stop_2_weight': (64, 64),
    'stop_2_bias': (64,),
    'stop_4_weight': (ACTION_DIMENSION, 64),
    'stop_4_bias': (ACTION_DIMENSION,),
}


def _silu(values):
    result = np.empty_like(values, dtype=np.float32)
    positive = values >= 0.0
    result[positive] = values[positive] / (
        1.0 + np.exp(-values[positive]))
    exponent = np.exp(values[~positive])
    result[~positive] = values[~positive] * exponent / (1.0 + exponent)
    return result


def _linear(values, weight, bias):
    return np.matmul(values, weight.T) + bias


class V12Policy:
    """Validated deterministic V12 actor without a PyTorch dependency."""

    def __init__(self, arrays):
        provided = set(arrays)
        expected = set(EXPECTED_ARRAY_SHAPES)
        if provided != expected:
            missing = sorted(expected - provided)
            unexpected = sorted(provided - expected)
            raise ValueError(
                f'invalid V12 policy arrays: missing={missing}, '
                f'unexpected={unexpected}')
        self.arrays = {}
        for name, shape in EXPECTED_ARRAY_SHAPES.items():
            values = np.asarray(arrays[name], dtype=np.float32)
            if values.shape != shape:
                raise ValueError(
                    f'V12 policy array {name!r} has shape {values.shape}, '
                    f'expected {shape}')
            if not np.isfinite(values).all():
                raise ValueError(
                    f'V12 policy array {name!r} contains non-finite values')
            values = values.copy()
            values.flags.writeable = False
            self.arrays[name] = values
        if np.any(self.arrays['normalizer_std'] < 0.0):
            raise ValueError('V12 normalizer standard deviation is negative')

    @classmethod
    def load(cls, path):
        """Load a non-pickled NPZ deployment artifact."""
        with np.load(Path(path), allow_pickle=False) as archive:
            arrays = {name: archive[name] for name in archive.files}
        return cls(arrays)

    def _branch(self, values, prefix, layer_indices):
        for layer_index in layer_indices[:-1]:
            values = _linear(
                values,
                self.arrays[f'{prefix}_{layer_index}_weight'],
                self.arrays[f'{prefix}_{layer_index}_bias'],
            )
            values = _silu(values)
        output_index = layer_indices[-1]
        return _linear(
            values,
            self.arrays[f'{prefix}_{output_index}_weight'],
            self.arrays[f'{prefix}_{output_index}_bias'],
        )

    def infer(self, observation):
        """Return deterministic actions for one observation or a batch."""
        observation = np.asarray(observation, dtype=np.float32)
        single = observation.ndim == 1
        if single:
            observation = observation.reshape(1, -1)
        if observation.ndim != 2 or observation.shape[1] != OBSERVATION_DIMENSION:
            raise ValueError(
                'V12 observation must have shape (78,) or (batch, 78)')
        if not np.isfinite(observation).all():
            raise ValueError('V12 observation contains non-finite values')

        latent = (
            observation - self.arrays['normalizer_mean']) / (
                self.arrays['normalizer_std'] + NORMALIZER_EPSILON)
        motion = self._branch(latent, 'motion', (0, 2, 4, 6, 8))
        stop_latent = latent.copy()
        stop_latent[:, CONTEXT_SLICE] = np.clip(
            observation[:, CONTEXT_SLICE] * CONTEXT_SCALE + CONTEXT_BIAS,
            -CONTEXT_CLIP,
            CONTEXT_CLIP,
        )
        stop = self._branch(stop_latent, 'stop', (0, 2, 4))
        exact_zero = np.all(observation[:, COMMAND_SLICE] == 0.0, axis=1)
        result = np.where(exact_zero[:, None], stop, motion).astype(
            np.float32, copy=False)
        if not np.isfinite(result).all():
            raise ValueError('V12 policy produced non-finite actions')
        return result[0].copy() if single else result
