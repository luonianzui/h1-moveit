"""Non-overwriting export and parity audit for the frozen V12 checkpoint."""

import argparse
import hashlib
from io import BytesIO
import json
from pathlib import Path
import shutil
import zipfile

import numpy as np

from .locomotion_contract import file_identity
from .locomotion_contract import load_contract
from .v12_policy import CONTEXT_BIAS
from .v12_policy import CONTEXT_SCALE
from .v12_policy import V12Policy


POLICY_FILE = 'v12_model_17250_policy.npz'
TORCHSCRIPT_FILE = 'v12_model_17250_policy.pt'
GOLDEN_FILE = 'v12_model_17250_golden.npz'
MANIFEST_FILE = 'v12_model_17250_manifest.json'


def _checkpoint_arrays(checkpoint_path):
    try:
        import torch
    except ImportError as error:
        raise RuntimeError('PyTorch is required only for V12 export') from error
    checkpoint = torch.load(
        checkpoint_path, map_location='cpu', weights_only=True)
    if checkpoint.get('iter') != 17250:
        raise ValueError('V12 checkpoint iteration must be 17250')
    state = checkpoint['actor_state_dict']
    mapping = {
        'normalizer_mean': 'obs_normalizer._mean',
        'normalizer_std': 'obs_normalizer._std',
    }
    for index in (0, 2, 4, 6, 8):
        mapping[f'motion_{index}_weight'] = f'mlp.{index}.weight'
        mapping[f'motion_{index}_bias'] = f'mlp.{index}.bias'
    for index in (0, 2, 4):
        mapping[f'stop_{index}_weight'] = f'zero_head.{index}.weight'
        mapping[f'stop_{index}_bias'] = f'zero_head.{index}.bias'
    arrays = {
        name: state[key].detach().cpu().numpy().astype(np.float32, copy=True)
        for name, key in mapping.items()
    }
    return arrays


def _torch_module(arrays):
    import torch
    import torch.nn as nn

    class ExportedV12Policy(nn.Module):
        def __init__(self):
            super().__init__()
            self.register_buffer(
                'normalizer_mean', torch.from_numpy(
                    arrays['normalizer_mean'].copy()))
            self.register_buffer(
                'normalizer_std', torch.from_numpy(
                    arrays['normalizer_std'].copy()))
            self.register_buffer(
                'context_scale', torch.from_numpy(CONTEXT_SCALE.copy()))
            self.register_buffer(
                'context_bias', torch.from_numpy(CONTEXT_BIAS.copy()))
            self.context_clip = 3.0
            self.motion = nn.Sequential(
                nn.Linear(78, 128), nn.SiLU(),
                nn.Linear(128, 128), nn.SiLU(),
                nn.Linear(128, 128), nn.SiLU(),
                nn.Linear(128, 128), nn.SiLU(),
                nn.Linear(128, 12),
            )
            self.stop = nn.Sequential(
                nn.Linear(78, 64), nn.SiLU(),
                nn.Linear(64, 64), nn.SiLU(),
                nn.Linear(64, 12),
            )
            self._load_branch(self.motion, 'motion', (0, 2, 4, 6, 8))
            self._load_branch(self.stop, 'stop', (0, 2, 4))

        def _load_branch(self, module, prefix, indices):
            for layer_index in indices:
                layer = module[layer_index]
                layer.weight.data.copy_(torch.from_numpy(
                    arrays[f'{prefix}_{layer_index}_weight']))
                layer.bias.data.copy_(torch.from_numpy(
                    arrays[f'{prefix}_{layer_index}_bias']))

        def forward(self, observation):
            exact_zero = torch.all(observation[:, 42:45] == 0.0, dim=-1)
            latent = (
                observation - self.normalizer_mean) / (
                    self.normalizer_std + 1.0e-2)
            motion = self.motion(latent)
            stop_latent = latent.clone()
            stop_context = (
                observation[:, 45:57] * self.context_scale
                + self.context_bias)
            stop_latent[:, 45:57] = torch.clamp(
                stop_context, -self.context_clip, self.context_clip)
            stop = self.stop(stop_latent)
            return torch.where(exact_zero[:, None], stop, motion)

        @torch.jit.export
        def reset(self):
            pass

    module = ExportedV12Policy()
    module.eval()
    return module


def _golden_observations():
    random = np.random.default_rng(17250)
    observation = random.normal(0.0, 1.0, (258, 78)).astype(np.float32)
    observation[:129, 42:45] = 0.0
    observation[129:, 42:45] = random.uniform(
        -0.3, 0.3, (129, 3)).astype(np.float32)
    observation[129, 42:45] = np.asarray(
        [np.nextafter(np.float32(0.0), np.float32(1.0)), 0.0, 0.0])
    observation[0] = 0.0
    return observation


def _parity_report(arrays):
    import torch

    module = _torch_module(arrays)
    observation = _golden_observations()
    with torch.inference_mode():
        expected = module(torch.from_numpy(observation)).numpy()
    actual = V12Policy(arrays).infer(observation)
    difference = np.abs(actual - expected)
    return {
        'observations': observation,
        'actions': expected.astype(np.float32),
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


def _source_digest():
    return hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def audit_export(contract):
    """Validate the checkpoint and compare NumPy inference with PyTorch."""
    expected = contract['policy']['checkpoint']
    actual = file_identity(expected['path'])
    if actual != {'size': expected['size'], 'sha256': expected['sha256']}:
        raise ValueError('V12 checkpoint identity mismatch')
    arrays = _checkpoint_arrays(expected['path'])
    policy = V12Policy(arrays)
    del policy
    parity = _parity_report(arrays)
    if parity['maximum_absolute_error'] > 5.0e-5:
        raise ValueError(
            'NumPy V12 parity exceeds 5e-5: '
            f"{parity['maximum_absolute_error']}")
    return arrays, parity


def build_export(contract, output_dir):
    """Create a new export directory without replacing any existing path."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=False, exist_ok=False)
    try:
        arrays, parity = audit_export(contract)
        policy_path = output_dir / POLICY_FILE
        torchscript_path = output_dir / TORCHSCRIPT_FILE
        golden_path = output_dir / GOLDEN_FILE
        manifest_path = output_dir / MANIFEST_FILE
        _deterministic_npz(policy_path, arrays)
        _deterministic_npz(golden_path, {
            'observations': parity['observations'],
            'actions': parity['actions'],
        })

        import torch
        scripted = torch.jit.script(_torch_module(arrays))
        torch.jit.save(scripted, torchscript_path)
        loaded = torch.jit.load(torchscript_path, map_location='cpu')
        with torch.inference_mode():
            scripted_actions = loaded(torch.from_numpy(
                parity['observations'])).numpy()
        script_error = float(np.max(np.abs(
            scripted_actions - parity['actions'])))
        if script_error != 0.0:
            raise ValueError(
                f'TorchScript export mismatch: {script_error}')

        manifest = {
            'schema_version': 1,
            'task': contract['policy']['task'],
            'iteration': contract['policy']['iteration'],
            'checkpoint': {
                **contract['policy']['checkpoint'],
            },
            'task_source_sha256': contract['assets']['task_source_sha256'],
            'exporter_sha256': _source_digest(),
            'policy': file_identity(policy_path),
            'torchscript': file_identity(torchscript_path),
            'golden': file_identity(golden_path),
            'golden_sample_count': int(parity['observations'].shape[0]),
            'numpy_maximum_absolute_error': (
                parity['maximum_absolute_error']),
            'numpy_mean_absolute_error': parity['mean_absolute_error'],
            'torchscript_maximum_absolute_error': script_error,
            'writes_performed': True,
        }
        with manifest_path.open('x', encoding='utf-8') as stream:
            json.dump(manifest, stream, indent=2, sort_keys=True)
            stream.write('\n')
        return manifest
    except BaseException:
        shutil.rmtree(output_dir)
        raise


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--contract', required=True)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--self-test', action='store_true')
    group.add_argument('--output-dir')
    parsed = parser.parse_args(args)
    contract = load_contract(parsed.contract)
    if parsed.self_test:
        _, parity = audit_export(contract)
        print(json.dumps({
            'checkpoint_valid': True,
            'golden_sample_count': int(parity['observations'].shape[0]),
            'maximum_absolute_error': parity['maximum_absolute_error'],
            'writes_performed': False,
        }, indent=2, sort_keys=True))
        return
    manifest = build_export(contract, parsed.output_dir)
    print(json.dumps(manifest, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
