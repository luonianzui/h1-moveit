"""Non-overwriting export and parity audit for the Unitree H1_2 actor."""

import argparse
import hashlib
from io import BytesIO
import json
from pathlib import Path
import zipfile

import numpy as np

from .unitree_h12_policy import UnitreeH12Policy


POLICY_FILE = 'unitree_h12_motion_policy.npz'
GOLDEN_FILE = 'unitree_h12_motion_golden.npz'
MANIFEST_FILE = 'unitree_h12_motion_manifest.json'
SOURCE_SHA256 = (
    'ea3003268d5d2e4b5004eb26d0969fbfdedced265ceeec7bf0b335309259cc80')
SOURCE_SIZE = 145745


def _file_identity(path):
    path = Path(path)
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return {'size': path.stat().st_size, 'sha256': digest.hexdigest()}


def _source_arrays(source_path):
    try:
        import torch
    except ImportError as error:
        raise RuntimeError(
            'PyTorch is required only for Unitree H1_2 export') from error
    source_path = Path(source_path)
    identity = _file_identity(source_path)
    if identity != {'size': SOURCE_SIZE, 'sha256': SOURCE_SHA256}:
        raise ValueError('official Unitree H1_2 source policy identity mismatch')
    module = torch.jit.load(str(source_path), map_location='cpu')
    state = module.state_dict()
    mapping = {
        'actor_0_weight': 'actor.0.weight',
        'actor_0_bias': 'actor.0.bias',
        'actor_2_weight': 'actor.2.weight',
        'actor_2_bias': 'actor.2.bias',
        'memory_weight_ih': 'memory.weight_ih_l0',
        'memory_weight_hh': 'memory.weight_hh_l0',
        'memory_bias_ih': 'memory.bias_ih_l0',
        'memory_bias_hh': 'memory.bias_hh_l0',
    }
    arrays = {
        name: state[key].detach().cpu().numpy().astype(np.float32, copy=True)
        for name, key in mapping.items()
    }
    UnitreeH12Policy(arrays)
    return module, arrays


def _golden_observations():
    random = np.random.default_rng(120047)
    observations = random.normal(0.0, 0.8, (257, 47)).astype(np.float32)
    observations[0] = 0.0
    observations[:, 3:6] *= np.float32(0.2)
    observations[:, 5] -= np.float32(1.0)
    observations[:, 6:9] = random.uniform(
        [-2.0, -2.0, -0.25], [2.0, 2.0, 0.25], (257, 3)).astype(
            np.float32)
    return observations


def _parity_report(module, arrays):
    import torch

    observations = _golden_observations()
    module.hidden_state.zero_()
    module.cell_state.zero_()
    expected = []
    with torch.inference_mode():
        for observation in observations:
            action = module(torch.from_numpy(observation).unsqueeze(0))
            expected.append(action.squeeze(0).numpy().copy())
    expected = np.asarray(expected, dtype=np.float32)
    policy = UnitreeH12Policy(arrays)
    actual = np.asarray([
        policy.infer(observation) for observation in observations
    ], dtype=np.float32)
    difference = np.abs(actual - expected)
    return {
        'observations': observations,
        'actions': expected,
        'maximum_absolute_error': float(difference.max()),
        'mean_absolute_error': float(difference.mean()),
    }


def _deterministic_npz(path, arrays):
    with Path(path).open('xb') as stream:
        with zipfile.ZipFile(
                stream, mode='w', compression=zipfile.ZIP_DEFLATED,
                compresslevel=9) as archive:
            for name in sorted(arrays):
                buffer = BytesIO()
                np.save(buffer, np.asarray(arrays[name]), allow_pickle=False)
                info = zipfile.ZipInfo(f'{name}.npy', (1980, 1, 1, 0, 0, 0))
                info.compress_type = zipfile.ZIP_DEFLATED
                info.external_attr = 0o100644 << 16
                archive.writestr(info, buffer.getvalue(), compresslevel=9)


def audit_export(source_path):
    """Validate the source actor and compare NumPy with TorchScript."""
    module, arrays = _source_arrays(source_path)
    parity = _parity_report(module, arrays)
    if parity['maximum_absolute_error'] > 5.0e-5:
        raise ValueError(
            'NumPy Unitree H1_2 parity exceeds 5e-5: '
            f"{parity['maximum_absolute_error']}")
    return arrays, parity


def build_export(source_path, output_dir):
    """Create export files without replacing any existing target."""
    output_dir = Path(output_dir)
    if not output_dir.is_dir():
        raise ValueError('output directory must already exist')
    targets = [
        output_dir / POLICY_FILE,
        output_dir / GOLDEN_FILE,
        output_dir / MANIFEST_FILE,
    ]
    if any(path.exists() for path in targets):
        raise FileExistsError('Unitree H1_2 export target already exists')
    created = []
    try:
        arrays, parity = audit_export(source_path)
        policy_path, golden_path, manifest_path = targets
        _deterministic_npz(policy_path, arrays)
        created.append(policy_path)
        _deterministic_npz(golden_path, {
            'observations': parity['observations'],
            'actions': parity['actions'],
        })
        created.append(golden_path)
        manifest = {
            'schema_version': 1,
            'source_policy': {
                'path': str(Path(source_path).resolve()),
                **_file_identity(source_path),
            },
            'exporter_sha256': hashlib.sha256(
                Path(__file__).read_bytes()).hexdigest(),
            'policy': _file_identity(policy_path),
            'golden': _file_identity(golden_path),
            'observation_dimension': 47,
            'action_dimension': 12,
            'recurrent_type': 'lstm',
            'recurrent_hidden_dimension': 64,
            'golden_sequence_length': int(
                parity['observations'].shape[0]),
            'numpy_maximum_absolute_error': (
                parity['maximum_absolute_error']),
            'numpy_mean_absolute_error': parity['mean_absolute_error'],
            'writes_performed': True,
        }
        with manifest_path.open('x', encoding='utf-8') as stream:
            json.dump(manifest, stream, indent=2, sort_keys=True)
            stream.write('\n')
        created.append(manifest_path)
        return manifest
    except BaseException:
        for path in reversed(created):
            path.unlink(missing_ok=True)
        raise


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', required=True)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--self-test', action='store_true')
    group.add_argument('--output-dir')
    parsed = parser.parse_args(args)
    if parsed.self_test:
        _, parity = audit_export(parsed.source)
        print(json.dumps({
            'source_valid': True,
            'golden_sequence_length': int(
                parity['observations'].shape[0]),
            'maximum_absolute_error': parity['maximum_absolute_error'],
            'writes_performed': False,
        }, indent=2, sort_keys=True))
        return
    manifest = build_export(parsed.source, parsed.output_dir)
    print(json.dumps(manifest, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
