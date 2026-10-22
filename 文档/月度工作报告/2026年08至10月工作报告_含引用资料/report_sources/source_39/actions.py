"""V12 STOP action with capture-pose anchoring and recovery hysteresis."""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from ...h1_2_human_velocity.mdp.actions import HumanIntegratedJointPositionAction
from ...h1_2_omni_context_stop_velocity.mdp.actions import (
    ContextStopHumanIntegratedJointPositionAction,
)

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv

    from .actions_cfg import RecoverStopHumanIntegratedJointPositionActionCfg


class RecoverStopHumanIntegratedJointPositionAction(
    ContextStopHumanIntegratedJointPositionAction
):
    """Brake around measured pose, hold a captured pose, and recover on drift.

    V8 permanently latched physical capture and then moved the target toward the
    default pose.  V12 instead requires a short stable dwell, snapshots the
    physically captured pose, and clears HOLD when velocity, attitude, or foot
    support leaves a wider hysteresis envelope.  The observable capture bit in
    the existing 12-D STOP context therefore remains the complete policy state.
    """

    cfg: RecoverStopHumanIntegratedJointPositionActionCfg

    def __init__(
        self,
        cfg: RecoverStopHumanIntegratedJointPositionActionCfg,
        env: ManagerBasedEnv,
    ) -> None:
        super().__init__(cfg, env)

        if cfg.capture_dwell_steps <= 0:
            raise ValueError("capture_dwell_steps must be positive.")
        positive_names = (
            "capture_angular_velocity_xy_threshold",
            "capture_vertical_velocity_threshold",
            "recovery_planar_speed_threshold",
            "recovery_yaw_speed_threshold",
            "recovery_angular_velocity_xy_threshold",
            "recovery_vertical_velocity_threshold",
        )
        for name in positive_names:
            value = getattr(cfg, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive.")
        if not math.isfinite(cfg.recovery_minimum_up_z) or not (
            -1.0 <= cfg.recovery_minimum_up_z <= cfg.capture_minimum_up_z
        ):
            raise ValueError(
                "recovery_minimum_up_z must be in [-1, capture_minimum_up_z]."
            )

        num_envs = self._env.num_envs
        device = self._env.device
        self._capture_stable_steps = torch.zeros(
            num_envs, dtype=torch.long, device=device
        )
        self._capture_step_observed = torch.full(
            (num_envs,), -1, dtype=torch.long, device=device
        )
        self._capture_targets = self._motor_targets.clone()
        self._capture_shoulder_targets = self._shoulder_targets.clone()

    def _torso_up_z(self) -> torch.Tensor:
        quat_w = self._asset.data.body_link_quat_w.torch[:, self._potential_body_id]
        return 1.0 - 2.0 * (
            torch.square(quat_w[:, 1]) + torch.square(quat_w[:, 2])
        )

    def _capture_candidate(self) -> torch.Tensor:
        angular_xy = torch.linalg.vector_norm(
            self._asset.data.root_ang_vel_b.torch[:, :2], dim=-1
        )
        vertical = torch.abs(self._asset.data.root_lin_vel_b.torch[:, 2])
        return (
            super()._capture_state()
            & (angular_xy < self.cfg.capture_angular_velocity_xy_threshold)
            & (vertical < self.cfg.capture_vertical_velocity_threshold)
        )

    def _capture_state(self) -> torch.Tensor:
        return self._capture_candidate() & (
            self._capture_stable_steps >= self.cfg.capture_dwell_steps
        )

    def _recovery_breach(self) -> torch.Tensor:
        root_vxy = self._asset.data.root_lin_vel_b.torch[:, :2]
        root_ang = self._asset.data.root_ang_vel_b.torch
        vertical = torch.abs(self._asset.data.root_lin_vel_b.torch[:, 2])
        left_contact = self._contact_state(self._left_contact_sensor)
        right_contact = self._contact_state(self._right_contact_sensor)
        return (
            (self._torso_up_z() < self.cfg.recovery_minimum_up_z)
            | (
                torch.linalg.vector_norm(root_vxy, dim=-1)
                > self.cfg.recovery_planar_speed_threshold
            )
            | (torch.abs(root_ang[:, 2]) > self.cfg.recovery_yaw_speed_threshold)
            | (
                torch.linalg.vector_norm(root_ang[:, :2], dim=-1)
                > self.cfg.recovery_angular_velocity_xy_threshold
            )
            | (vertical > self.cfg.recovery_vertical_velocity_threshold)
            | (~left_contact)
            | (~right_contact)
        )

    def refresh_stop_context(self) -> torch.Tensor:
        """Update stable dwell/recovery once per physical step, then encode context."""
        command = self._env.command_manager.get_command(self.cfg.velocity_command_name)
        if command.shape != (self._env.num_envs, 3):
            raise ValueError(
                f"Command {self.cfg.velocity_command_name!r} must have shape "
                f"(num_envs, 3), received {tuple(command.shape)}."
            )

        episode_step = self._env.episode_length_buf.to(dtype=torch.long)
        advanced = episode_step != self._capture_step_observed
        zero = torch.all(command == 0.0, dim=-1)
        moving = ~zero
        candidate = self._capture_candidate()

        increment = advanced & zero & candidate & (~self._capture_latched)
        clear_dwell = advanced & (moving | (~candidate))
        self._capture_stable_steps[increment] += 1
        self._capture_stable_steps[clear_dwell] = 0
        self._capture_step_observed[advanced] = episode_step[advanced]

        breached = zero & self._capture_latched & self._recovery_breach()
        self._capture_latched[breached] = False
        self._capture_stable_steps[breached] = 0

        previously_latched = self._capture_latched.clone()
        context = super().refresh_stop_context()
        newly_latched = self._capture_latched & (~previously_latched)
        if torch.any(newly_latched):
            self._capture_targets[newly_latched] = self._asset.data.joint_pos.torch[
                newly_latched
            ][:, self._joint_ids]
            self._capture_shoulder_targets[newly_latched] = (
                self._asset.data.joint_pos.torch[newly_latched][
                    :, self._shoulder_joint_ids
                ]
            )
        return context

    def process_actions(self, actions: torch.Tensor) -> None:
        """Apply the V8 STOP residual around measured or captured pose."""
        command = self._env.command_manager.get_command(self.cfg.velocity_command_name)
        if command.shape != (self._env.num_envs, 3):
            raise ValueError(
                f"Command {self.cfg.velocity_command_name!r} must have shape "
                f"(num_envs, 3), received {tuple(command.shape)}."
            )

        self.refresh_stop_context()
        self._cache_pre_step_potential()

        previous_motor_targets = self._motor_targets.clone()
        previous_shoulder_targets = self._shoulder_targets.clone()
        HumanIntegratedJointPositionAction.process_actions(self, actions)
        parent_motor_targets = self._motor_targets.clone()
        parent_shoulder_targets = self._shoulder_targets.clone()

        zero = torch.all(command == 0.0, dim=-1)
        moving = ~zero
        entering_stop = zero & (self._stop_state == self.MOVE)

        self._stop_state[moving] = self.MOVE
        self._capture_latched[moving] = False
        self._capture_stable_steps[moving] = 0
        self._stop_targets[moving] = parent_motor_targets[moving]
        self._stop_shoulder_targets[moving] = parent_shoulder_targets[moving]
        self._stop_age[moving] = 0

        self._stop_state[entering_stop] = self.STOP
        self._stop_targets[entering_stop] = previous_motor_targets[entering_stop]
        self._stop_shoulder_targets[entering_stop] = previous_shoulder_targets[
            entering_stop
        ]
        self._stop_age[entering_stop] = 0
        self._capture_latched[entering_stop] = False
        self._capture_stable_steps[entering_stop] = 0

        actual_targets = self._asset.data.joint_pos.torch[:, self._joint_ids]
        captured = zero & self._capture_latched
        reference = torch.where(
            captured[:, None], self._capture_targets, actual_targets
        )
        desired = reference + self._stop_residual_scale * torch.tanh(self._raw_actions)
        desired = torch.clamp(
            desired,
            min=self._target_limits[..., 0],
            max=self._target_limits[..., 1],
        )
        target_delta = torch.clamp(
            desired - self._stop_targets,
            min=-self._stop_rate_limit,
            max=self._stop_rate_limit,
        )
        next_stop_targets = self._stop_targets + target_delta
        self._stop_targets[zero] = next_stop_targets[zero]
        self._stop_age[zero] += 1

        shoulder_reference = torch.where(
            captured[:, None],
            self._capture_shoulder_targets,
            self._stop_shoulder_targets,
        )
        shoulder_delta = torch.clamp(
            shoulder_reference - self._stop_shoulder_targets,
            min=-self.cfg.stop_shoulder_rate_limit,
            max=self.cfg.stop_shoulder_rate_limit,
        )
        next_stop_shoulders = self._stop_shoulder_targets + shoulder_delta
        self._stop_shoulder_targets[zero] = next_stop_shoulders[zero]
        self._stop_shoulder_targets[:] = torch.clamp(
            self._stop_shoulder_targets,
            min=self._shoulder_limits[..., 0],
            max=self._shoulder_limits[..., 1],
        )

        self._motor_targets[:] = torch.where(
            zero[:, None], self._stop_targets, parent_motor_targets
        )
        self._processed_actions[:] = self._motor_targets
        self._shoulder_targets[:] = torch.where(
            zero[:, None], self._stop_shoulder_targets, parent_shoulder_targets
        )

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        if env_ids is None:
            env_ids = slice(None)
        super().reset(env_ids)
        self._capture_stable_steps[env_ids] = 0
        self._capture_step_observed[env_ids] = -1
        self._capture_targets[env_ids] = self._motor_targets[env_ids]
        self._capture_shoulder_targets[env_ids] = self._shoulder_targets[env_ids]


__all__ = ["RecoverStopHumanIntegratedJointPositionAction"]
