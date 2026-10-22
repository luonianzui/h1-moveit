"""Frozen V2 motion actor with a manually scaled V8 STOP context."""

from __future__ import annotations

import copy
import math

import torch
import torch.nn as nn
from tensordict import TensorDict

from rsl_rl.utils import unpad_trajectories

from ...h1_2_omni_arrest_velocity.agents.frozen_motion_adapter import (
    FrozenMotionPPO,
    FrozenMotionStopAdapterModel,
    GatedGaussianDistribution,
)


class FrozenMotionContextStopAdapterModel(FrozenMotionStopAdapterModel):
    """Keep frozen V2 normalization for motion and scale STOP context by hand.

    The raw conditional context occupies policy indices ``45:57`` only at an
    exact-zero command.  The V2 normalizer was trained on joint velocities at
    those indices, so the STOP branch overwrites that normalized slice with a
    fixed, documented affine scaling.  The frozen motion MLP always receives
    the original V2-normalized latent tensor.
    """

    def __init__(
        self,
        *args,
        context_start: int = 45,
        context_end: int = 57,
        context_scale: tuple[float, ...] | list[float] = (
            2.0 / 3.0,
            4.0 / 3.0,
            2.0 / 3.0,
            4.0 / 3.0,
            2.0 / 3.0,
            2.0,
            1.0,
            1.0,
            1.0,
            1.0,
            2.0,
            2.0,
        ),
        context_bias: tuple[float, ...] | list[float] = (
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            -1.0,
            -1.0,
        ),
        context_clip: float = 3.0,
        **kwargs,
    ) -> None:
        super().__init__(*args, **kwargs)
        if context_start < 0 or context_end != context_start + 12:
            raise ValueError(
                "V8 STOP context must select twelve values; received "
                f"[{context_start}:{context_end}]."
            )
        if len(context_scale) != 12 or len(context_bias) != 12:
            raise ValueError("context_scale and context_bias must each contain 12 values.")
        if any((not math.isfinite(value)) or value == 0.0 for value in context_scale):
            raise ValueError("context_scale must contain finite non-zero values.")
        if any(not math.isfinite(value) for value in context_bias):
            raise ValueError("context_bias must contain finite values.")
        if not math.isfinite(context_clip) or context_clip <= 0.0:
            raise ValueError("context_clip must be finite and positive.")

        self.context_start = context_start
        self.context_end = context_end
        self.context_clip = context_clip
        # Scaling is part of the task ABI, not learned checkpoint state.  Making
        # these buffers non-persistent preserves strict legacy V2 migration.
        self.register_buffer(
            "_context_scale",
            torch.tensor(context_scale, dtype=torch.float32).unsqueeze(0),
            persistent=False,
        )
        self.register_buffer(
            "_context_bias",
            torch.tensor(context_bias, dtype=torch.float32).unsqueeze(0),
            persistent=False,
        )

    def _stop_latent(
        self, policy_obs: torch.Tensor, normalized_latent: torch.Tensor
    ) -> torch.Tensor:
        stop_latent = normalized_latent.clone()
        scaled_context = (
            policy_obs[:, self.context_start : self.context_end]
            * self._context_scale.to(dtype=policy_obs.dtype)
            + self._context_bias.to(dtype=policy_obs.dtype)
        )
        stop_latent[:, self.context_start : self.context_end] = torch.clamp(
            scaled_context,
            min=-self.context_clip,
            max=self.context_clip,
        )
        return stop_latent

    def forward(
        self,
        obs: TensorDict,
        masks: torch.Tensor | None = None,
        hidden_state=None,
        stochastic_output: bool = False,
    ) -> torch.Tensor:
        if masks is not None and not self.is_recurrent:
            obs = unpad_trajectories(obs, masks)
        policy_obs = obs["policy"]
        zero_mask = self._exact_zero_mask(obs)
        latent = self.get_latent(obs, masks=None, hidden_state=hidden_state)
        motion_mean = self.mlp(latent)
        zero_mean = self.zero_head(self._stop_latent(policy_obs, latent))
        mixed_mean = torch.where(zero_mask[:, None], zero_mean, motion_mean)

        if stochastic_output:
            self.distribution.set_zero_mask(zero_mask)
            self.distribution.update(mixed_mean)
            return self.distribution.sample()
        return mixed_mean

    def as_jit(self) -> nn.Module:
        return _FrozenMotionContextStopAdapterJit(self)


class _FrozenMotionContextStopAdapterJit(nn.Module):
    """TorchScript deterministic export with identical STOP preprocessing."""

    def __init__(self, model: FrozenMotionContextStopAdapterModel) -> None:
        super().__init__()
        self.obs_normalizer = copy.deepcopy(model.obs_normalizer)
        self.mlp = copy.deepcopy(model.mlp)
        self.zero_head = copy.deepcopy(model.zero_head)
        self.command_start = model.command_start
        self.command_end = model.command_end
        self.context_start = model.context_start
        self.context_end = model.context_end
        self.context_clip = model.context_clip
        self.register_buffer("context_scale", model._context_scale.detach().clone())
        self.register_buffer("context_bias", model._context_bias.detach().clone())

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        zero_mask = torch.all(
            x[:, self.command_start : self.command_end] == 0.0, dim=-1
        )
        latent = self.obs_normalizer(x)
        motion_mean = self.mlp(latent)
        stop_latent = latent.clone()
        scaled_context = (
            x[:, self.context_start : self.context_end] * self.context_scale
            + self.context_bias
        )
        stop_latent[:, self.context_start : self.context_end] = torch.clamp(
            scaled_context, min=-self.context_clip, max=self.context_clip
        )
        zero_mean = self.zero_head(stop_latent)
        return torch.where(zero_mask[:, None], zero_mean, motion_mean)

    @torch.jit.export
    def reset(self) -> None:
        pass


class ContextStopPPO(FrozenMotionPPO):
    """V2-migration PPO with bounded, trainable exact-zero exploration."""

    def __init__(
        self,
        *args,
        zero_std_min: float = 0.12,
        zero_std_max: float = 0.35,
        **kwargs,
    ) -> None:
        if not (0.0 < zero_std_min <= zero_std_max):
            raise ValueError("Require 0 < zero_std_min <= zero_std_max.")
        self._zero_log_std_min = math.log(zero_std_min)
        self._zero_log_std_max = math.log(zero_std_max)
        super().__init__(*args, **kwargs)
        self._clamp_zero_std()

    def _clamp_zero_std(self) -> None:
        with torch.no_grad():
            self.actor.distribution.zero_log_std_param.clamp_(
                min=self._zero_log_std_min,
                max=self._zero_log_std_max,
            )

    def update(self, *args, **kwargs) -> dict[str, float]:
        """Clamp around an otherwise version-compatible PPO update call."""
        self._clamp_zero_std()
        losses = super().update(*args, **kwargs)
        self._clamp_zero_std()
        return losses

    def load(self, loaded_dict: dict, load_cfg: dict | None, strict: bool) -> bool:
        load_iteration = super().load(loaded_dict, load_cfg, strict)
        self._clamp_zero_std()
        return load_iteration


__all__ = [
    "ContextStopPPO",
    "FrozenMotionContextStopAdapterModel",
    "GatedGaussianDistribution",
]
