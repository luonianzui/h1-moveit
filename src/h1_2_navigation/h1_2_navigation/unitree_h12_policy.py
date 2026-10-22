"""Pure NumPy inference for the official Unitree H1_2 recurrent actor."""

from pathlib import Path

import numpy as np


OBSERVATION_DIMENSION = 47
ACTION_DIMENSION = 12
HIDDEN_DIMENSION = 64
ACTOR_HIDDEN_DIMENSION = 32

EXPECTED_ARRAY_SHAPES = {
    'actor_0_weight': (ACTOR_HIDDEN_DIMENSION, HIDDEN_DIMENSION),
    'actor_0_bias': (ACTOR_HIDDEN_DIMENSION,),
    'actor_2_weight': (ACTION_DIMENSION, ACTOR_HIDDEN_DIMENSION),
    'actor_2_bias': (ACTION_DIMENSION,),
    'memory_weight_ih': (4 * HIDDEN_DIMENSION, OBSERVATION_DIMENSION),
    'memory_weight_hh': (4 * HIDDEN_DIMENSION, HIDDEN_DIMENSION),
    'memory_bias_ih': (4 * HIDDEN_DIMENSION,),
    'memory_bias_hh': (4 * HIDDEN_DIMENSION,),
}


def _linear(values, weight, bias):
    return np.matmul(values, weight.T) + bias


def _sigmoid(values):
    result = np.empty_like(values, dtype=np.float32)
    positive = values >= 0.0
    result[positive] = 1.0 / (1.0 + np.exp(-values[positive]))
    exponent = np.exp(values[~positive])
    result[~positive] = exponent / (1.0 + exponent)
    return result


def _elu(values):
    return np.where(values > 0.0, values, np.expm1(values)).astype(
        np.float32, copy=False)


class UnitreeH12Policy:
    """Validated stateful actor without a runtime PyTorch dependency."""

    def __init__(self, arrays):
        provided = set(arrays)
        expected = set(EXPECTED_ARRAY_SHAPES)
        if provided != expected:
            missing = sorted(expected - provided)
            unexpected = sorted(provided - expected)
            raise ValueError(
                'invalid Unitree H1_2 policy arrays: '
                f'missing={missing}, unexpected={unexpected}')
        self.arrays = {}
        for name, shape in EXPECTED_ARRAY_SHAPES.items():
            values = np.asarray(arrays[name], dtype=np.float32)
            if values.shape != shape:
                raise ValueError(
                    f'Unitree H1_2 policy array {name!r} has shape '
                    f'{values.shape}, expected {shape}')
            if not np.isfinite(values).all():
                raise ValueError(
                    f'Unitree H1_2 policy array {name!r} contains '
                    'non-finite values')
            values = values.copy()
            values.flags.writeable = False
            self.arrays[name] = values
        self.hidden_state = np.zeros(HIDDEN_DIMENSION, dtype=np.float32)
        self.cell_state = np.zeros(HIDDEN_DIMENSION, dtype=np.float32)

    @classmethod
    def load(cls, path):
        """Load a non-pickled NPZ deployment artifact."""
        with np.load(Path(path), allow_pickle=False) as archive:
            arrays = {name: archive[name] for name in archive.files}
        return cls(arrays)

    def reset(self):
        """Reset the recurrent state for a new locomotion episode."""
        self.hidden_state.fill(0.0)
        self.cell_state.fill(0.0)

    def infer(self, observation):
        """Advance one recurrent step and return twelve leg actions."""
        observation = np.asarray(observation, dtype=np.float32)
        if observation.shape != (OBSERVATION_DIMENSION,):
            raise ValueError('Unitree H1_2 observation must have shape (47,)')
        if not np.isfinite(observation).all():
            raise ValueError(
                'Unitree H1_2 observation contains non-finite values')
        gates = (
            _linear(
                observation,
                self.arrays['memory_weight_ih'],
                self.arrays['memory_bias_ih'])
            + _linear(
                self.hidden_state,
                self.arrays['memory_weight_hh'],
                self.arrays['memory_bias_hh']))
        input_gate, forget_gate, cell_gate, output_gate = np.split(gates, 4)
        input_gate = _sigmoid(input_gate)
        forget_gate = _sigmoid(forget_gate)
        cell_gate = np.tanh(cell_gate).astype(np.float32, copy=False)
        output_gate = _sigmoid(output_gate)
        self.cell_state = (
            forget_gate * self.cell_state + input_gate * cell_gate).astype(
                np.float32, copy=False)
        self.hidden_state = (
            output_gate * np.tanh(self.cell_state)).astype(
                np.float32, copy=False)
        latent = _elu(_linear(
            self.hidden_state,
            self.arrays['actor_0_weight'],
            self.arrays['actor_0_bias']))
        action = _linear(
            latent,
            self.arrays['actor_2_weight'],
            self.arrays['actor_2_bias']).astype(np.float32, copy=False)
        if not np.isfinite(action).all():
            raise ValueError('Unitree H1_2 policy produced non-finite actions')
        return action.copy()
