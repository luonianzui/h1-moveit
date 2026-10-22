#!/usr/bin/env python3
"""Batch-evaluate motion-to-stop transitions for H1_2 omni policies.

The default workload is 256 environments by 2000 control steps.  Environments
are divided evenly (difference at most one) among the 12 non-zero commands from
``eval_h12_omni_checkpoint.py``.  The workload is split into independent,
synchronously reset 1000-step attempts.  Each attempt uses the assigned motion
command for 500 steps followed by a stop request for 500 steps.  By default the
request is an abrupt exact ``[0, 0, 0]`` command.  Optional acceleration limits
can slew the effective command down and commit exact zero only during confirmed
double support.  The last 100 steps must always be exact zero and are reported
separately.

``ManagerBasedRLEnv.step`` automatically resets done environments before it
returns.  To prevent those reset trajectories from making an unstable stop look
healthy, every attempt maintains a monotonic ``intact`` cohort mask.  Once an
environment terminates unexpectedly or becomes non-finite, none of its later
states in that attempt are counted.  A successful attempt must reach the final
step without an earlier reset and end with a pure time-out (no simultaneous
failure termination).  The active phase command is also installed in the
command term's reset resampler so it cannot be replaced during automatic reset.
"""

from __future__ import annotations

import argparse
import copy
import importlib.metadata as metadata
import math
import traceback
import types
from pathlib import Path

import h1_2_isaaclab
from isaaclab.app import AppLauncher


DEFAULT_TASK = "H12-Human-Omni-Gait-Flat-v2"

# These are exactly the non-zero scenarios from eval_h12_omni_checkpoint.py.
# That evaluator has 13 total scenarios: one stand command plus these 12.
MOTION_SCENARIOS: tuple[tuple[str, float, float, float], ...] = (
    ("forward", 0.5, 0.0, 0.0),
    ("backward", -0.5, 0.0, 0.0),
    ("lateral_left", 0.0, 0.35, 0.0),
    ("lateral_right", 0.0, -0.35, 0.0),
    ("diagonal_forward_left", 0.4, 0.3, 0.0),
    ("diagonal_forward_right", 0.4, -0.3, 0.0),
    ("diagonal_backward_left", -0.4, 0.3, 0.0),
    ("diagonal_backward_right", -0.4, -0.3, 0.0),
    ("yaw_left", 0.0, 0.0, 0.6),
    ("yaw_right", 0.0, 0.0, -0.6),
    ("forward_yaw_left", 0.5, 0.0, 0.6),
    ("forward_yaw_right", 0.5, 0.0, -0.6),
)

h1_2_isaaclab.register_tasks()

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("checkpoint_path", type=Path, help="RSL-RL model_*.pt checkpoint")
parser.add_argument("--task", default=DEFAULT_TASK, help=f"Gym task id (default: {DEFAULT_TASK})")
parser.add_argument("--num-envs", "--num_envs", dest="num_envs", type=int, default=256)
parser.add_argument("--num-steps", "--num_steps", dest="num_steps", type=int, default=2000)
parser.add_argument("--episode-length", "--episode_length", dest="episode_length", type=int, default=1000)
parser.add_argument("--motion-steps", type=int, default=500, help="motion steps per command cycle")
parser.add_argument("--stop-steps", type=int, default=500, help="zero-command steps per cycle")
parser.add_argument(
    "--motion-command-scale",
    type=float,
    default=1.0,
    help="multiply all 12 non-zero motion commands by this value in (0, 1]",
)
parser.add_argument(
    "--max-motion-planar-rmse",
    type=float,
    default=0.35,
    help="per-attempt maximum planar tracking RMSE in m/s (default: 0.35)",
)
parser.add_argument(
    "--max-motion-yaw-rmse",
    type=float,
    default=0.45,
    help="per-attempt maximum yaw tracking RMSE in rad/s (default: 0.45)",
)
parser.add_argument(
    "--max-tail-planar-rms",
    type=float,
    default=0.10,
    help="per-attempt maximum final-tail planar speed RMS in m/s (default: 0.10)",
)
parser.add_argument(
    "--max-tail-yaw-rms",
    type=float,
    default=0.12,
    help="per-attempt maximum final-tail yaw speed RMS in rad/s (default: 0.12)",
)
parser.add_argument(
    "--stop-tail-steps",
    type=int,
    default=100,
    help="last stop-phase steps reported separately (default: 100)",
)
parser.add_argument("--seed", type=int, default=0)
parser.add_argument(
    "--linear-deceleration",
    type=float,
    default=0.0,
    help="planar command deceleration in m/s^2; zero keeps the abrupt-stop test",
)
parser.add_argument(
    "--yaw-deceleration",
    type=float,
    default=0.0,
    help="yaw command deceleration in rad/s^2; zero keeps the abrupt-stop test",
)
parser.add_argument(
    "--contact-aligned-zero",
    action="store_true",
    help="after slewing down, submit exact zero only during confirmed double support",
)
parser.add_argument(
    "--contact-confirm-steps",
    type=int,
    default=3,
    help="consecutive double-support control steps before exact zero (default: 3)",
)
parser.add_argument(
    "--minimum-command-scale",
    type=float,
    default=0.10,
    help="non-zero fraction of the motion command held while awaiting double support",
)
parser.add_argument(
    "--zero-release-planar-threshold",
    type=float,
    default=0.15,
    help="maximum effective planar command at which double-support zero may engage",
)
parser.add_argument(
    "--zero-release-yaw-threshold",
    type=float,
    default=0.15,
    help="maximum effective yaw command at which double-support zero may engage",
)
parser.add_argument(
    "--zero-release-actual-planar-speed",
    type=float,
    default=0.12,
    help="maximum measured base planar speed before exact zero may engage",
)
parser.add_argument(
    "--zero-release-actual-yaw-speed",
    type=float,
    default=0.15,
    help="maximum measured base yaw speed before exact zero may engage",
)
parser.add_argument(
    "--zero-command-target-leak",
    type=float,
    default=None,
    help=(
        "diagnostic override for actions.joint_pos.zero_command_target_leak; "
        "requires --allow-config-mismatch when it differs from the checkpoint"
    ),
)
parser.add_argument(
    "--zero-command-anchor-leak",
    type=float,
    default=None,
    help=(
        "diagnostic override for actions.joint_pos.zero_command_anchor_leak; "
        "requires --allow-config-mismatch when it differs from the checkpoint"
    ),
)
parser.add_argument(
    "--allow-config-mismatch",
    action="store_true",
    help="evaluate despite a missing/different training configuration snapshot",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
import yaml  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg  # noqa: E402
from isaaclab_tasks.utils.parse_cfg import load_cfg_from_registry, parse_env_cfg  # noqa: E402


ENV_SNAPSHOT_KEYS = (
    "sim",
    "scene",
    "observations",
    "actions",
    "commands",
    "rewards",
    "terminations",
    "events",
    "curriculum",
    "decimation",
    "episode_length_s",
    "is_finite_horizon",
)
PROVENANCE_SNAPSHOT_KEYS = (
    "task_source_sha256",
    "robot_asset_sha256",
    "runtime_versions",
)
AGENT_SNAPSHOT_KEYS = (
    "class_name",
    "actor",
    "critic",
    "algorithm",
    "clip_actions",
    "obs_groups",
    "num_steps_per_env",
)


def _normalize_snapshot(env_snapshot: dict, agent_snapshot: dict) -> tuple[dict, dict]:
    """Remove only evaluation-time fields allowed to differ from training."""
    env_snapshot = copy.deepcopy(env_snapshot)
    agent_snapshot = copy.deepcopy(agent_snapshot)

    env_snapshot["sim"].pop("device", None)
    scene = env_snapshot["scene"]
    scene.pop("num_envs", None)
    scene["terrain"].pop("num_envs", None)
    scene["terrain"].pop("env_spacing", None)

    for entity_name in ("robot", "left_foot_contact", "right_foot_contact"):
        prim_path = scene[entity_name]["prim_path"]
        scene[entity_name]["prim_path"] = prim_path.replace(
            "/World/envs/env_.*/", "{ENV_REGEX_NS}/"
        )
    for entity_name in ("robot", "sky_light"):
        scene[entity_name]["spawn"].pop("spawn_path", None)

    normalized_env = {key: env_snapshot[key] for key in ENV_SNAPSHOT_KEYS}
    normalized_env.update(
        {key: env_snapshot[key] for key in PROVENANCE_SNAPSHOT_KEYS if key in env_snapshot}
    )
    return normalized_env, {key: agent_snapshot[key] for key in AGENT_SNAPSHOT_KEYS}


def _validate_run_snapshot(checkpoint_path: Path, env_cfg, agent_cfg) -> None:
    """Require task configuration and source provenance to match training."""
    params_dir = checkpoint_path.parent / "params"
    env_yaml = params_dir / "env.yaml"
    agent_yaml = params_dir / "agent.yaml"
    missing = [str(path) for path in (env_yaml, agent_yaml) if not path.is_file()]
    if missing:
        message = "checkpoint 缺少训练配置快照: " + ", ".join(missing)
        if args_cli.allow_config_mismatch:
            print(f"[WARNING] {message}", flush=True)
            return
        raise FileNotFoundError(message + "；如确需绕过，请显式加 --allow-config-mismatch")

    with env_yaml.open(encoding="utf-8") as stream:
        saved_env = yaml.unsafe_load(stream)
    with agent_yaml.open(encoding="utf-8") as stream:
        saved_agent = yaml.unsafe_load(stream)

    current_env, current_agent = _normalize_snapshot(env_cfg.to_dict(), agent_cfg.to_dict())
    saved_env, saved_agent = _normalize_snapshot(saved_env, saved_agent)
    mismatches = [
        *(f"env.{key}" for key in ENV_SNAPSHOT_KEYS if current_env[key] != saved_env[key]),
        *(f"agent.{key}" for key in AGENT_SNAPSHOT_KEYS if current_agent[key] != saved_agent[key]),
    ]
    missing_provenance = [key for key in PROVENANCE_SNAPSHOT_KEYS if key not in saved_env]
    if missing_provenance:
        print(
            "[WARNING] 训练快照未带完整来源哈希，只能校验已有配置: "
            + ", ".join(missing_provenance),
            flush=True,
        )
    mismatches.extend(
        f"env.{key}"
        for key in PROVENANCE_SNAPSHOT_KEYS
        if key in saved_env and current_env[key] != saved_env[key]
    )
    if mismatches:
        message = "当前源码配置与 checkpoint 训练快照不一致: " + ", ".join(mismatches)
        if args_cli.allow_config_mismatch:
            print(f"[WARNING] {message}", flush=True)
            return
        raise ValueError(message + "；为避免误评已停止，如确需跨配置评估请显式加 --allow-config-mismatch")

    print(f"训练配置快照校验通过: {params_dir}", flush=True)


def _scenario_assignment(num_envs: int, device: str) -> tuple[torch.Tensor, torch.Tensor]:
    """Return a balanced fixed motion-scenario assignment."""
    scenario_ids = torch.arange(num_envs, device=device, dtype=torch.long) % len(MOTION_SCENARIOS)
    table = torch.tensor(
        [[vx, vy, yaw] for _, vx, vy, yaw in MOTION_SCENARIOS],
        device=device,
        dtype=torch.float32,
    ) * args_cli.motion_command_scale
    motion_targets = table[scenario_ids]
    counts = torch.bincount(scenario_ids, minlength=len(MOTION_SCENARIOS))
    if int(counts.max().item() - counts.min().item()) > 1:
        raise AssertionError("scenario groups are not balanced")
    if torch.any(torch.linalg.vector_norm(motion_targets, dim=-1) <= 1.0e-6):
        raise AssertionError("motion scenario table unexpectedly contains a zero command")
    return scenario_ids, motion_targets


class PhaseLockedCommands:
    """Lock the current motion/stop command across automatic environment resets."""

    def __init__(self, base_env, initial_targets: torch.Tensor):
        self.env = base_env
        self.targets = initial_targets.clone()
        self.term = base_env.command_manager.get_term("base_velocity")
        if not hasattr(self.term, "vel_command_b"):
            raise TypeError(
                "base_velocity term does not expose vel_command_b; transition evaluation is unsafe"
            )

        controller = self

        def _fixed_resample(term, env_ids) -> None:
            term.vel_command_b[env_ids] = controller.targets[env_ids]
            standing = torch.linalg.vector_norm(controller.targets[env_ids], dim=-1) <= 1.0e-6
            if hasattr(term, "is_standing_env"):
                term.is_standing_env[env_ids] = standing
            if hasattr(term, "is_heading_env"):
                term.is_heading_env[env_ids] = False

        def _fixed_update(term) -> None:
            term.vel_command_b.copy_(controller.targets)
            standing = torch.linalg.vector_norm(controller.targets, dim=-1) <= 1.0e-6
            if hasattr(term, "is_standing_env"):
                term.is_standing_env.copy_(standing)
            if hasattr(term, "is_heading_env"):
                term.is_heading_env.zero_()

        self.term._resample_command = types.MethodType(_fixed_resample, self.term)
        self.term._update_command = types.MethodType(_fixed_update, self.term)
        self.force()

    def set_targets(self, targets: torch.Tensor) -> bool:
        """Set a phase target and return whether observations must be refreshed."""
        if targets.shape != self.targets.shape:
            raise ValueError(f"unexpected command shape: {targets.shape} != {self.targets.shape}")
        changed = not torch.equal(targets, self.targets)
        self.targets.copy_(targets)
        self.force()
        return changed

    def force(self) -> None:
        """Defensively restore current commands and refresh derived gait."""
        self.term.vel_command_b.copy_(self.targets)
        self.term.time_left.fill_(1.0e9)
        standing = torch.linalg.vector_norm(self.targets, dim=-1) <= 1.0e-6
        if hasattr(self.term, "is_standing_env"):
            self.term.is_standing_env.copy_(standing)
        if hasattr(self.term, "is_heading_env"):
            self.term.is_heading_env.zero_()
        if "gait" in self.env.command_manager.active_terms:
            self.env.command_manager.get_term("gait")._update_command()

    def assert_locked(self) -> None:
        actual = self.env.command_manager.get_command("base_velocity")
        # Fixed table commands compare bitwise.  Slewed commands may pass
        # through one base-frame update and differ by a few float32 ulps.
        if not torch.allclose(actual, self.targets, rtol=1.0e-6, atol=1.0e-5):
            max_error = torch.max(torch.abs(actual - self.targets)).item()
            raise RuntimeError(f"phase command was overwritten (max error={max_error:g})")


class SmoothStopCommands:
    """Slew motion commands to zero and optionally align zero with double support."""

    def __init__(self, base_env, motion_targets: torch.Tensor):
        self.env = base_env
        self.motion_targets = motion_targets
        self.effective = motion_targets.clone()
        self.zero_released = torch.zeros(
            base_env.num_envs, device=base_env.device, dtype=torch.bool
        )
        self.awaiting_contact = torch.zeros_like(self.zero_released)
        self.contact_steps = torch.zeros(
            base_env.num_envs, device=base_env.device, dtype=torch.int32
        )
        if args_cli.contact_aligned_zero:
            try:
                self.left_sensor = base_env.scene.sensors["left_foot_contact"]
                self.right_sensor = base_env.scene.sensors["right_foot_contact"]
            except KeyError as error:
                raise KeyError(
                    "--contact-aligned-zero requires left_foot_contact and "
                    "right_foot_contact sensors"
                ) from error
        else:
            self.left_sensor = None
            self.right_sensor = None

    def reset(self) -> None:
        self.effective.copy_(self.motion_targets)
        self.zero_released.zero_()
        self.awaiting_contact.zero_()
        self.contact_steps.zero_()

    @staticmethod
    def _contact(sensor, threshold: float = 5.0) -> torch.Tensor:
        forces = sensor.data.net_forces_w.torch
        return (forces.norm(dim=-1) > threshold).any(dim=1)

    def step_toward_zero(self) -> torch.Tensor:
        """Return the next effective command for a zero requested command."""
        if args_cli.linear_deceleration == 0.0 and args_cli.yaw_deceleration == 0.0:
            candidate = torch.zeros_like(self.effective)
        else:
            candidate = self.effective.clone()
            planar = candidate[:, :2]
            planar_norm = torch.linalg.vector_norm(planar, dim=-1, keepdim=True)
            planar_next = torch.clamp(
                planar_norm - args_cli.linear_deceleration * self.env.step_dt,
                min=0.0,
            )
            planar_scale = torch.where(
                planar_norm > 0.0,
                planar_next / torch.clamp(planar_norm, min=1.0e-12),
                torch.zeros_like(planar_norm),
            )
            candidate[:, :2] = planar * planar_scale

            yaw = candidate[:, 2]
            yaw_next = torch.clamp(
                torch.abs(yaw) - args_cli.yaw_deceleration * self.env.step_dt,
                min=0.0,
            )
            candidate[:, 2] = torch.sign(yaw) * yaw_next

        if not args_cli.contact_aligned_zero:
            self.effective.copy_(candidate)
            return self.effective

        planar_small = (
            torch.linalg.vector_norm(candidate[:, :2], dim=-1)
            <= args_cli.zero_release_planar_threshold
        )
        yaw_small = torch.abs(candidate[:, 2]) <= args_cli.zero_release_yaw_threshold
        actual_planar_small = (
            torch.linalg.vector_norm(
                self.env.scene["robot"].data.root_lin_vel_b.torch[:, :2], dim=-1
            )
            <= args_cli.zero_release_actual_planar_speed
        )
        actual_yaw_small = (
            torch.abs(self.env.scene["robot"].data.root_ang_vel_b.torch[:, 2])
            <= args_cli.zero_release_actual_yaw_speed
        )
        self.awaiting_contact |= planar_small & yaw_small & ~self.zero_released
        awaiting_contact = self.awaiting_contact & ~self.zero_released
        double_contact = self._contact(self.left_sensor) & self._contact(self.right_sensor)
        release_ready = awaiting_contact & actual_planar_small & actual_yaw_small
        self.contact_steps[:] = torch.where(
            release_ready & double_contact,
            self.contact_steps + 1,
            torch.zeros_like(self.contact_steps),
        )
        self.zero_released |= awaiting_contact & (
            self.contact_steps >= args_cli.contact_confirm_steps
        )
        self.awaiting_contact &= ~self.zero_released
        hold_nonzero = awaiting_contact & ~self.zero_released
        candidate[hold_nonzero] = (
            self.motion_targets[hold_nonzero] * args_cli.minimum_command_scale
        )
        candidate[self.zero_released] = 0.0
        self.effective.copy_(candidate)
        return self.effective


class TransitionMetrics:
    """Independent-attempt survival, first-failure, and velocity statistics."""

    _SYNTHETIC_FAILURE_CAUSES = (
        "unexpected_time_out",
        "nonfinite_state",
        "motion_insufficient_samples",
        "motion_planar_tracking",
        "motion_yaw_tracking",
        "missing_expected_time_out",
        "late_zero_release",
        "zero_command_not_reached",
        "tail_insufficient_samples",
        "tail_planar_speed",
        "tail_yaw_speed",
        "unclassified_termination",
        "unclassified_done",
    )

    def __init__(
        self,
        base_env,
        scenario_ids: torch.Tensor,
        termination_names: tuple[str, ...],
        *,
        num_cycles: int,
        motion_steps: int,
        cycle_steps: int,
    ):
        self.env = base_env
        self.device = base_env.device
        self.ids = scenario_ids
        self.n = len(MOTION_SCENARIOS)
        self.robot = base_env.scene["robot"]
        self.termination_names = termination_names
        self.num_cycles = num_cycles
        self.motion_steps = motion_steps
        self.cycle_steps = cycle_steps
        self.env_counts = torch.bincount(scenario_ids, minlength=self.n).to(torch.float64)
        self.non_timeout_term_indices = tuple(
            index
            for index, name in enumerate(termination_names)
            if not base_env.termination_manager.get_term_cfg(name).time_out
        )

        def zeros() -> torch.Tensor:
            return torch.zeros(self.n, device=self.device, dtype=torch.float64)

        # State metrics. Motion and stop use only the original cohort up to its
        # first failure. Tail metrics are stricter: only complete successful
        # attempts are committed at the end of a cycle.
        self.motion_valid = zeros()
        self.motion_vx_error_sq = zeros()
        self.motion_vy_error_sq = zeros()
        self.motion_yaw_error_sq = zeros()
        self.stop_valid = zeros()
        self.stop_vx_sq = zeros()
        self.stop_vy_sq = zeros()
        self.stop_yaw_sq = zeros()
        self.tail_valid = zeros()
        self.tail_vx_sq = zeros()
        self.tail_vy_sq = zeros()
        self.tail_yaw_sq = zeros()

        # Per-attempt acceptance records. A row is one synchronously reset cycle.
        attempt_shape = (num_cycles, base_env.num_envs)
        self.motion_survived = torch.zeros(attempt_shape, device=self.device, dtype=torch.bool)
        self.success = torch.zeros_like(self.motion_survived)
        self.first_failure_step = torch.full(
            attempt_shape, -1, device=self.device, dtype=torch.int32
        )
        # 0=motion, 1=stop. -1 means no failure.
        self.first_failure_phase = torch.full(
            attempt_shape, -1, device=self.device, dtype=torch.int8
        )
        # One-based cycle step of the first literal exact-zero command.  A
        # negative value means zero was never reached by an intact attempt.
        self.zero_release_step = torch.full(
            attempt_shape, -1, device=self.device, dtype=torch.int32
        )
        # One-based cycle step at which a task-provided capture latch is first
        # observed true before the policy action.  This is diagnostic only and
        # never participates in acceptance.  Tasks without a real
        # capture-latch tensor leave this table unused.
        self.first_capture_step = torch.full(
            attempt_shape, -1, device=self.device, dtype=torch.int32
        )
        self.capture_latched_supported = False
        self.first_failure_causes = {
            name: zeros()
            for name in (
                *(termination_names[index] for index in self.non_timeout_term_indices),
                *self._SYNTHETIC_FAILURE_CAUSES,
            )
        }

        # Raw auto-reset event statistics are retained only as a diagnostic
        # appendix. They are not used as an acceptance denominator.
        self.done_events = zeros()
        self.early_done_events = zeros()
        self.episode_length_done_events = zeros()
        self.event_term_counts = {name: zeros() for name in termination_names}

        self.intact = torch.zeros(base_env.num_envs, device=self.device, dtype=torch.bool)
        self._current_cycle = -1
        self._motion_boundary_recorded = False
        self._cycle_tail_valid = torch.zeros(
            base_env.num_envs, device=self.device, dtype=torch.float64
        )
        self._cycle_tail_vx_sq = torch.zeros_like(self._cycle_tail_valid)
        self._cycle_tail_vy_sq = torch.zeros_like(self._cycle_tail_valid)
        self._cycle_tail_yaw_sq = torch.zeros_like(self._cycle_tail_valid)
        self._cycle_motion_valid = torch.zeros_like(self._cycle_tail_valid)
        self._cycle_motion_vx_error_sq = torch.zeros_like(self._cycle_tail_valid)
        self._cycle_motion_vy_error_sq = torch.zeros_like(self._cycle_tail_valid)
        self._cycle_motion_yaw_error_sq = torch.zeros_like(self._cycle_tail_valid)

    def _scatter(self, destination: torch.Tensor, values: torch.Tensor) -> None:
        destination.scatter_add_(0, self.ids, values.to(torch.float64))

    def begin_cycle(self, cycle_id: int) -> None:
        """Start a new synchronized cohort after the caller reset every env."""
        if cycle_id != self._current_cycle + 1:
            raise RuntimeError(f"non-sequential cycle id: {cycle_id} after {self._current_cycle}")
        self._current_cycle = cycle_id
        self._motion_boundary_recorded = False
        self.intact.fill_(True)
        self._cycle_tail_valid.zero_()
        self._cycle_tail_vx_sq.zero_()
        self._cycle_tail_vy_sq.zero_()
        self._cycle_tail_yaw_sq.zero_()
        self._cycle_motion_valid.zero_()
        self._cycle_motion_vx_error_sq.zero_()
        self._cycle_motion_vy_error_sq.zero_()
        self._cycle_motion_yaw_error_sq.zero_()

    def mark_motion_boundary(self, cycle_id: int) -> None:
        """Record which original attempts reached the first zero-command step."""
        if cycle_id != self._current_cycle or self._motion_boundary_recorded:
            raise RuntimeError(f"invalid motion boundary for cycle {cycle_id}")
        count = torch.clamp(self._cycle_motion_valid, min=1.0)
        planar_rmse = torch.sqrt(
            (self._cycle_motion_vx_error_sq + self._cycle_motion_vy_error_sq) / count
        )
        yaw_rmse = torch.sqrt(self._cycle_motion_yaw_error_sq / count)
        insufficient = self.intact & (self._cycle_motion_valid < self.motion_steps)
        planar_failure = (
            self.intact
            & ~insufficient
            & (planar_rmse > args_cli.max_motion_planar_rmse)
        )
        yaw_failure = (
            self.intact
            & ~insufficient
            & (yaw_rmse > args_cli.max_motion_yaw_rmse)
        )
        quality_failure = insufficient | planar_failure | yaw_failure
        self.motion_survived[cycle_id].copy_(self.intact & ~quality_failure)
        if quality_failure.any():
            self.first_failure_step[cycle_id, quality_failure] = self.motion_steps
            self.first_failure_phase[cycle_id, quality_failure] = 0
            self._scatter(
                self.first_failure_causes["motion_insufficient_samples"], insufficient
            )
            self._scatter(
                self.first_failure_causes["motion_planar_tracking"], planar_failure
            )
            self._scatter(
                self.first_failure_causes["motion_yaw_tracking"], yaw_failure
            )
            self.intact &= ~quality_failure
        self._motion_boundary_recorded = True

    def record_capture_latched(
        self,
        cycle_id: int,
        cycle_index: int,
        capture_latched: torch.Tensor,
    ) -> None:
        """Record first observed latch step for the still-intact cohort.

        The observation is taken before the policy action for ``cycle_index``.
        It is deliberately read-only with respect to the environment and does
        not alter success, failure, or cohort masks.
        """
        if cycle_id != self._current_cycle:
            raise RuntimeError(f"capture observation for inactive cycle {cycle_id}")
        if capture_latched.shape != self.intact.shape:
            raise ValueError(
                "unexpected capture_latched shape: "
                f"{capture_latched.shape} != {self.intact.shape}"
            )
        first_capture = (
            self.intact
            & capture_latched.bool()
            & (self.first_capture_step[cycle_id] < 0)
        )
        self.first_capture_step[cycle_id, first_capture] = cycle_index + 1

    def update_step(
        self,
        cycle_id: int,
        cycle_index: int,
        done: torch.Tensor,
        time_out: torch.Tensor,
        terminated: torch.Tensor,
        term_matrix: torch.Tensor,
        command_during_step: torch.Tensor,
        *,
        motion_phase: bool,
        tail_phase: bool,
    ) -> None:
        """Update one step without ever re-admitting an auto-reset trajectory."""
        if cycle_id != self._current_cycle:
            raise RuntimeError(f"step for inactive cycle {cycle_id}")

        lin_vel = self.robot.data.root_lin_vel_b.torch
        ang_vel = self.robot.data.root_ang_vel_b.torch
        finite = (
            torch.isfinite(lin_vel).all(dim=-1)
            & torch.isfinite(ang_vel).all(dim=-1)
            & torch.isfinite(command_during_step).all(dim=-1)
        )
        intact_before = self.intact.clone()
        final_step = cycle_index == self.cycle_steps - 1
        exact_zero = torch.all(command_during_step == 0.0, dim=-1)
        first_zero = (
            intact_before
            & (not motion_phase)
            & exact_zero
            & (self.zero_release_step[cycle_id] < 0)
        )
        self.zero_release_step[cycle_id, first_zero] = cycle_index + 1

        # Only a pure time-out on the final step is a successful completion.
        if final_step:
            expected_time_out = done & time_out & ~terminated
        else:
            expected_time_out = torch.zeros_like(done)
        unexpected_done = done & ~expected_time_out
        nonfinite_failure = ~done & ~finite
        # A successful attempt must spend the complete configured tail under a
        # literal exact-zero command.  Checking only the final step could admit
        # a controller that releases zero too late to demonstrate a stable
        # stationary hold.
        late_zero_release = intact_before & tail_phase & ~exact_zero
        zero_command_not_reached = torch.zeros_like(done)
        if final_step:
            zero_command_not_reached = intact_before & torch.any(
                command_during_step != 0.0, dim=-1
            )
        tail_insufficient_samples = torch.zeros_like(done)
        tail_planar_speed = torch.zeros_like(done)
        tail_yaw_speed = torch.zeros_like(done)
        if final_step:
            expected_tail_samples = max(1, args_cli.stop_tail_steps - 1)
            tail_insufficient_samples = intact_before & (
                self._cycle_tail_valid < expected_tail_samples
            )
            tail_count = torch.clamp(self._cycle_tail_valid, min=1.0)
            tail_planar_rms = torch.sqrt(
                (self._cycle_tail_vx_sq + self._cycle_tail_vy_sq) / tail_count
            )
            tail_yaw_rms = torch.sqrt(self._cycle_tail_yaw_sq / tail_count)
            tail_planar_speed = (
                intact_before
                & ~tail_insufficient_samples
                & (tail_planar_rms > args_cli.max_tail_planar_rms)
            )
            tail_yaw_speed = (
                intact_before
                & ~tail_insufficient_samples
                & (tail_yaw_rms > args_cli.max_tail_yaw_rms)
            )
        failure = intact_before & (
            unexpected_done
            | nonfinite_failure
            | late_zero_release
            | zero_command_not_reached
            | tail_insufficient_samples
            | tail_planar_speed
            | tail_yaw_speed
        )

        # A synchronized attempt must time out exactly at its final step. Treat
        # a missing timeout as a failure instead of silently calling it alive.
        missing_expected_time_out = torch.zeros_like(done)
        if final_step:
            missing_expected_time_out = intact_before & ~failure & ~expected_time_out
            failure |= missing_expected_time_out

        if failure.any():
            self.first_failure_step[cycle_id, failure] = cycle_index + 1
            self.first_failure_phase[cycle_id, failure] = 0 if motion_phase else 1

            covered = torch.zeros_like(done)
            for term_index in self.non_timeout_term_indices:
                cause = failure & term_matrix[:, term_index]
                self._scatter(
                    self.first_failure_causes[self.termination_names[term_index]], cause
                )
                covered |= cause

            unexpected_time_out = failure & time_out & ~terminated & ~expected_time_out
            self._scatter(
                self.first_failure_causes["unexpected_time_out"], unexpected_time_out
            )
            covered |= unexpected_time_out

            nonfinite_cause = failure & nonfinite_failure
            self._scatter(self.first_failure_causes["nonfinite_state"], nonfinite_cause)
            covered |= nonfinite_cause

            self._scatter(
                self.first_failure_causes["missing_expected_time_out"],
                missing_expected_time_out,
            )
            covered |= missing_expected_time_out

            self._scatter(
                self.first_failure_causes["late_zero_release"],
                failure & late_zero_release,
            )
            covered |= failure & late_zero_release

            self._scatter(
                self.first_failure_causes["zero_command_not_reached"],
                failure & zero_command_not_reached,
            )
            covered |= failure & zero_command_not_reached

            self._scatter(
                self.first_failure_causes["tail_insufficient_samples"],
                failure & tail_insufficient_samples,
            )
            covered |= failure & tail_insufficient_samples

            self._scatter(
                self.first_failure_causes["tail_planar_speed"],
                failure & tail_planar_speed,
            )
            covered |= failure & tail_planar_speed

            self._scatter(
                self.first_failure_causes["tail_yaw_speed"],
                failure & tail_yaw_speed,
            )
            covered |= failure & tail_yaw_speed

            unclassified_termination = failure & terminated & ~covered
            self._scatter(
                self.first_failure_causes["unclassified_termination"],
                unclassified_termination,
            )
            covered |= unclassified_termination

            unclassified_done = failure & done & ~covered
            self._scatter(self.first_failure_causes["unclassified_done"], unclassified_done)

        valid = intact_before & ~failure & ~done & finite
        valid_f = valid.to(lin_vel.dtype)
        if motion_phase:
            self._scatter(self.motion_valid, valid)
            valid_64 = valid.to(torch.float64)
            self._cycle_motion_valid += valid_64
            self._cycle_motion_vx_error_sq += (
                (lin_vel[:, 0] - command_during_step[:, 0]).square().to(torch.float64)
                * valid_64
            )
            self._cycle_motion_vy_error_sq += (
                (lin_vel[:, 1] - command_during_step[:, 1]).square().to(torch.float64)
                * valid_64
            )
            self._cycle_motion_yaw_error_sq += (
                (ang_vel[:, 2] - command_during_step[:, 2]).square().to(torch.float64)
                * valid_64
            )
            self._scatter(
                self.motion_vx_error_sq,
                (lin_vel[:, 0] - command_during_step[:, 0]).square() * valid_f,
            )
            self._scatter(
                self.motion_vy_error_sq,
                (lin_vel[:, 1] - command_during_step[:, 1]).square() * valid_f,
            )
            self._scatter(
                self.motion_yaw_error_sq,
                (ang_vel[:, 2] - command_during_step[:, 2]).square() * valid_f,
            )
        else:
            self._scatter(self.stop_valid, valid)
            self._scatter(self.stop_vx_sq, lin_vel[:, 0].square() * valid_f)
            self._scatter(self.stop_vy_sq, lin_vel[:, 1].square() * valid_f)
            self._scatter(self.stop_yaw_sq, ang_vel[:, 2].square() * valid_f)
            if tail_phase:
                valid_64 = valid.to(torch.float64)
                self._cycle_tail_valid += valid_64
                self._cycle_tail_vx_sq += lin_vel[:, 0].square().to(torch.float64) * valid_64
                self._cycle_tail_vy_sq += lin_vel[:, 1].square().to(torch.float64) * valid_64
                self._cycle_tail_yaw_sq += ang_vel[:, 2].square().to(torch.float64) * valid_64

        self.intact &= ~failure

        if final_step:
            successful = intact_before & expected_time_out & ~failure
            self.success[cycle_id].copy_(successful)
            tail_observed = (
                intact_before
                & expected_time_out
                & ~tail_insufficient_samples
                & ~late_zero_release
                & ~zero_command_not_reached
            )
            tail_observed_64 = tail_observed.to(torch.float64)
            self._scatter(self.tail_valid, self._cycle_tail_valid * tail_observed_64)
            self._scatter(self.tail_vx_sq, self._cycle_tail_vx_sq * tail_observed_64)
            self._scatter(self.tail_vy_sq, self._cycle_tail_vy_sq * tail_observed_64)
            self._scatter(self.tail_yaw_sq, self._cycle_tail_yaw_sq * tail_observed_64)

    def update_event_stats(
        self,
        done: torch.Tensor,
        episode_steps: torch.Tensor,
        term_matrix: torch.Tensor,
        episode_length: int,
    ) -> None:
        """Retain raw done events as a diagnostic appendix, never acceptance."""
        early = done & (episode_steps < episode_length)
        reached_length = done & ~early
        self._scatter(self.done_events, done)
        self._scatter(self.early_done_events, early)
        self._scatter(self.episode_length_done_events, reached_length)
        for term_index, name in enumerate(self.termination_names):
            self._scatter(self.event_term_counts[name], term_matrix[:, term_index])

    def finish_cycle(self, cycle_id: int) -> None:
        """Assert every synchronized attempt has one acceptance outcome."""
        if cycle_id != self._current_cycle or not self._motion_boundary_recorded:
            raise RuntimeError(f"cycle {cycle_id} finished before its motion boundary")
        classified = self.success[cycle_id] | (self.first_failure_step[cycle_id] > 0)
        if not torch.all(classified).item():
            missing = int(torch.count_nonzero(~classified).item())
            raise RuntimeError(f"cycle {cycle_id} has {missing} unclassified attempts")
        if torch.any(self.success[cycle_id] & ~self.motion_survived[cycle_id]).item():
            raise RuntimeError(f"cycle {cycle_id} has success without motion survival")

    @staticmethod
    def _rms(total: float, count: float) -> float:
        return math.sqrt(max(0.0, total / count)) if count > 0 else float("nan")

    @staticmethod
    def _pct(numerator: int, denominator: int) -> float:
        return 100.0 * numerator / denominator if denominator else float("nan")

    @staticmethod
    def _failure_step_summary(values: torch.Tensor) -> str:
        values = values[values > 0]
        if values.numel() == 0:
            return "n/a"
        values_f = values.to(torch.float64)
        return (
            f"{int(values.min().item())}/"
            f"{values_f.mean().item():.1f}/"
            f"{int(values.max().item())}"
        )

    def _scenario_attempt_counts(self, index: int) -> tuple[int, int, int, int, int]:
        scenario_mask = self.ids == index
        attempts = int(torch.count_nonzero(scenario_mask).item()) * self.num_cycles
        motion_ok = int(torch.count_nonzero(self.motion_survived[:, scenario_mask]).item())
        success = int(torch.count_nonzero(self.success[:, scenario_mask]).item())
        phases = self.first_failure_phase[:, scenario_mask]
        motion_fail = int(torch.count_nonzero(phases == 0).item())
        stop_fail = int(torch.count_nonzero(phases == 1).item())
        return attempts, motion_ok, success, motion_fail, stop_fail

    def report(self) -> None:
        print("\n=== 运动→停止独立 attempt 验收 ===")
        print(
            "口径: 每周期同步 reset；失败后的自动重置轨迹永久排除。周期末仅纯 time_out "
            "且运动跟踪和停止尾速均通过阈值才算成功；首次失败原因是 multi-label"
            "（同一步可触发多个 term）。"
        )
        print(
            "质量阈值: motion planar/yaw RMSE <= "
            f"{args_cli.max_motion_planar_rmse:g} m/s / "
            f"{args_cli.max_motion_yaw_rmse:g} rad/s；tail planar/yaw RMS <= "
            f"{args_cli.max_tail_planar_rms:g} m/s / "
            f"{args_cli.max_tail_yaw_rms:g} rad/s"
        )
        print(
            "scenario                       cmd[vx,vy,yaw]       attempts motion_ok success "
            "motion% stop|motion% e2e% fail[m/s] first_step[min/mean/max]"
        )
        for index, (name, vx, vy, yaw) in enumerate(MOTION_SCENARIOS):
            vx *= args_cli.motion_command_scale
            vy *= args_cli.motion_command_scale
            yaw *= args_cli.motion_command_scale
            attempts, motion_ok, success, motion_fail, stop_fail = self._scenario_attempt_counts(index)
            scenario_mask = self.ids == index
            first_steps = self.first_failure_step[:, scenario_mask]
            motion_n = self.motion_valid[index].item()
            stop_n = self.stop_valid[index].item()
            tail_n = self.tail_valid[index].item()
            print(
                f"{name:<30} [{vx:+.2f},{vy:+.2f},{yaw:+.2f}]  "
                f"{attempts:>8} {motion_ok:>9} {success:>7} "
                f"{self._pct(motion_ok, attempts):>7.1f} "
                f"{self._pct(success, motion_ok):>12.1f} "
                f"{self._pct(success, attempts):>5.1f} "
                f"[{motion_fail:>2}/{stop_fail:<2}] "
                f"{self._failure_step_summary(first_steps):>24}"
            )
            print(
                "  motion intact-cohort RMSE [vx,vy,yaw]="
                f"[{self._rms(self.motion_vx_error_sq[index].item(), motion_n):.4f}, "
                f"{self._rms(self.motion_vy_error_sq[index].item(), motion_n):.4f}, "
                f"{self._rms(self.motion_yaw_error_sq[index].item(), motion_n):.4f}] "
                f"(n={int(motion_n)})"
            )
            print(
                "  stop intact-until-failure RMS [vx,vy,yaw]="
                f"[{self._rms(self.stop_vx_sq[index].item(), stop_n):.4f}, "
                f"{self._rms(self.stop_vy_sq[index].item(), stop_n):.4f}, "
                f"{self._rms(self.stop_yaw_sq[index].item(), stop_n):.4f}] "
                f"(n={int(stop_n)}); completed-attempt last {args_cli.stop_tail_steps}="
                f"[{self._rms(self.tail_vx_sq[index].item(), tail_n):.4f}, "
                f"{self._rms(self.tail_vy_sq[index].item(), tail_n):.4f}, "
                f"{self._rms(self.tail_yaw_sq[index].item(), tail_n):.4f}] "
                f"(n={int(tail_n)})"
            )
            causes = ", ".join(
                f"{cause}={int(values[index].item())}"
                for cause, values in self.first_failure_causes.items()
                if values[index].item() > 0
            )
            print(f"  first_failure_causes: {causes or 'none'}")

        attempts = self.num_cycles * self.env.num_envs
        motion_ok = int(torch.count_nonzero(self.motion_survived).item())
        success = int(torch.count_nonzero(self.success).item())
        motion_fail = int(torch.count_nonzero(self.first_failure_phase == 0).item())
        stop_fail = int(torch.count_nonzero(self.first_failure_phase == 1).item())
        motion_n = self.motion_valid.sum().item()
        stop_n = self.stop_valid.sum().item()
        tail_n = self.tail_valid.sum().item()
        print("\n=== 验收汇总 ===")
        print(
            f"独立 attempts={attempts}, motion_survival={motion_ok}/{attempts} "
            f"({self._pct(motion_ok, attempts):.2f}%), conditional_stop_survival={success}/{motion_ok} "
            f"({self._pct(success, motion_ok):.2f}%), end_to_end={success}/{attempts} "
            f"({self._pct(success, attempts):.2f}%), first_fail[motion/stop]={motion_fail}/{stop_fail}"
        )
        print(
            "运动阶段 intact-cohort 总体 RMSE [vx,vy,yaw]="
            f"[{self._rms(self.motion_vx_error_sq.sum().item(), motion_n):.4f}, "
            f"{self._rms(self.motion_vy_error_sq.sum().item(), motion_n):.4f}, "
            f"{self._rms(self.motion_yaw_error_sq.sum().item(), motion_n):.4f}]"
        )
        print(
            "停止阶段 intact-until-failure 总体实际 RMS [vx,vy,yaw]="
            f"[{self._rms(self.stop_vx_sq.sum().item(), stop_n):.4f}, "
            f"{self._rms(self.stop_vy_sq.sum().item(), stop_n):.4f}, "
            f"{self._rms(self.stop_yaw_sq.sum().item(), stop_n):.4f}]；"
            f"完成且可观测 attempts 最后 {args_cli.stop_tail_steps} 步="
            f"[{self._rms(self.tail_vx_sq.sum().item(), tail_n):.4f}, "
            f"{self._rms(self.tail_vy_sq.sum().item(), tail_n):.4f}, "
            f"{self._rms(self.tail_yaw_sq.sum().item(), tail_n):.4f}] "
            f"(tail_n={int(tail_n)})"
        )
        causes = ", ".join(
            f"{cause}={int(values.sum().item())}"
            for cause, values in self.first_failure_causes.items()
            if values.sum().item() > 0
        )
        print(f"首次失败原因汇总 (multi-label): {causes or 'none'}")
        released = self.zero_release_step[self.zero_release_step > 0].to(torch.float64)
        if released.numel() > 0:
            print(
                "exact-zero 首次释放 cycle_step[min/mean/max]="
                f"[{int(released.min().item())}/{released.mean().item():.1f}/"
                f"{int(released.max().item())}]；完整停止 tail 必须从 "
                f"step {self.cycle_steps - args_cli.stop_tail_steps + 1} 起逐步为零"
            )
        else:
            print("exact-zero 首次释放 cycle_step: none")

        if self.capture_latched_supported:
            captured = self.first_capture_step > 0
            captured_steps = self.first_capture_step[captured].to(torch.float64)
            captured_then_failed = (
                captured
                & (self.first_failure_step > 0)
                & (self.first_failure_step >= self.first_capture_step)
            )
            capture_failure_latency = (
                self.first_failure_step[captured_then_failed]
                - self.first_capture_step[captured_then_failed]
            ).to(torch.float64)
            if captured_steps.numel() > 0:
                stop_steps = captured_steps - self.motion_steps
                first_capture_summary = (
                    f"[{int(stop_steps.min().item())}/{stop_steps.mean().item():.1f}/"
                    f"{int(stop_steps.max().item())}]"
                )
            else:
                first_capture_summary = "none"
            if capture_failure_latency.numel() > 0:
                latency_summary = (
                    f"[{int(capture_failure_latency.min().item())}/"
                    f"{capture_failure_latency.mean().item():.1f}/"
                    f"{int(capture_failure_latency.max().item())}]"
                )
            else:
                latency_summary = "n/a"
            print(
                "capture_latched 只读诊断: 首次观测 stop_step[min/mean/max]="
                f"{first_capture_summary}, captured_attempts="
                f"{int(torch.count_nonzero(captured).item())}/{attempts}, "
                "captured_then_failed="
                f"{int(torch.count_nonzero(captured_then_failed).item())}, "
                "capture→first_failure latency[min/mean/max]="
                f"{latency_summary}"
            )
        else:
            print(
                "capture_latched 只读诊断: unavailable "
                "(任务 action term 未暴露 capture_latched latch tensor；已安全跳过)"
            )

        print("\n=== 原始自动重置事件附录（不用于验收分母） ===")
        for index, (name, _, _, _) in enumerate(MOTION_SCENARIOS):
            terms = ", ".join(
                f"{term}={int(values[index].item())}"
                for term, values in self.event_term_counts.items()
                if values[index].item() > 0
            )
            print(
                f"{name:<30} done_events={int(self.done_events[index].item())}, "
                f"early_events={int(self.early_done_events[index].item())}, "
                f"episode_length_events={int(self.episode_length_done_events[index].item())}, "
                f"terms: {terms or 'none'}"
            )


def _evaluate_checkpoint(env, agent_cfg, checkpoint_path: Path) -> None:
    if env.max_episode_length != args_cli.episode_length:
        raise ValueError(
            "episode_length 与任务配置不一致: "
            f"命令行={args_cli.episode_length}, 环境={env.max_episode_length}。"
        )

    print(f"加载 checkpoint: {checkpoint_path}", flush=True)
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(str(checkpoint_path))
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    base_env = env.unwrapped
    scenario_ids, motion_targets = _scenario_assignment(base_env.num_envs, base_env.device)
    command_lock = PhaseLockedCommands(base_env, motion_targets)
    smooth_stop = SmoothStopCommands(base_env, motion_targets)
    command_lock.assert_locked()
    termination_names = tuple(base_env.termination_manager.active_terms)

    cycle_steps = args_cli.motion_steps + args_cli.stop_steps
    num_cycles = args_cli.num_steps // cycle_steps
    metrics = TransitionMetrics(
        base_env,
        scenario_ids,
        termination_names,
        num_cycles=num_cycles,
        motion_steps=args_cli.motion_steps,
        cycle_steps=cycle_steps,
    )
    print(
        f"任务 {args_cli.task}；{args_cli.num_envs} env × {args_cli.num_steps} 步，"
        f"episode_length={args_cli.episode_length}；{num_cycles} 个同步独立周期，每周期 "
        f"{args_cli.motion_steps} 步运动 + {args_cli.stop_steps} 步停止请求；"
        f"12 个非零运动场景；motion_command_scale={args_cli.motion_command_scale:g}",
        flush=True,
    )
    if args_cli.linear_deceleration > 0.0:
        print(
            "停止控制: "
            f"planar_decel={args_cli.linear_deceleration:g} m/s^2, "
            f"yaw_decel={args_cli.yaw_deceleration:g} rad/s^2, "
            f"contact_aligned_zero={args_cli.contact_aligned_zero}, "
            f"contact_confirm={args_cli.contact_confirm_steps}, "
            f"release_thresholds=({args_cli.zero_release_planar_threshold:g}, "
            f"{args_cli.zero_release_yaw_threshold:g}), "
            f"actual_speed_thresholds=({args_cli.zero_release_actual_planar_speed:g}, "
            f"{args_cli.zero_release_actual_yaw_speed:g})",
            flush=True,
        )
    else:
        print("停止控制: abrupt exact zero", flush=True)
    print(f"活动终止项: {', '.join(termination_names)}", flush=True)
    for index, (name, vx, vy, yaw) in enumerate(MOTION_SCENARIOS):
        vx *= args_cli.motion_command_scale
        vy *= args_cli.motion_command_scale
        yaw *= args_cli.motion_command_scale
        count = int((scenario_ids == index).sum().item())
        print(f"  {name}: n={count}, motion=[{vx:+.2f}, {vy:+.2f}, {yaw:+.2f}]", flush=True)

    episode_steps = torch.zeros(args_cli.num_envs, dtype=torch.int32, device=base_env.device)
    reset_dones = torch.ones(args_cli.num_envs, dtype=torch.long, device=base_env.device)
    progress_interval = max(1, args_cli.num_steps // 4)
    completed_steps = 0
    action_term = base_env.action_manager.get_term("joint_pos")
    stop_action_sensitivity_supported = all(
        hasattr(action_term, name)
        for name in (
            "_asset",
            "_joint_ids",
            "_stop_targets",
            "_stop_residual_scale",
            "_stop_rate_limit",
        )
    )
    capture_latched_attribute = next(
        (
            name
            for name in ("capture_latched", "_capture_latched")
            if isinstance(getattr(action_term, name, None), torch.Tensor)
        ),
        None,
    )
    capture_latched_supported = capture_latched_attribute is not None
    metrics.capture_latched_supported = capture_latched_supported

    with torch.inference_mode():
        for cycle_id in range(num_cycles):
            # Every cycle is a fresh, synchronized transition attempt. Set the
            # motion target before reset so the replacement resampler and the
            # reset observation both use the correct command.
            command_lock.set_targets(motion_targets)
            smooth_stop.reset()
            obs, _ = env.reset()
            command_lock.force()
            command_lock.assert_locked()
            if hasattr(policy, "reset"):
                policy.reset(reset_dones)
            # Recompute after the defensive force so the first action sees the
            # assigned motion command even if reset internals change later.
            obs = env.get_observations()
            episode_steps.zero_()
            metrics.begin_cycle(cycle_id)
            capture_previous = None
            capture_toggles = torch.zeros(
                args_cli.num_envs, dtype=torch.int32, device=base_env.device
            )

            for cycle_index in range(cycle_steps):
                motion_phase = cycle_index < args_cli.motion_steps
                if cycle_index == args_cli.motion_steps:
                    metrics.mark_motion_boundary(cycle_id)
                tail_phase = (
                    not motion_phase
                    and cycle_index >= cycle_steps - args_cli.stop_tail_steps
                )
                phase_targets = (
                    motion_targets if motion_phase else smooth_stop.step_toward_zero()
                )
                changed = command_lock.set_targets(phase_targets)
                command_lock.assert_locked()
                if changed:
                    # env.step's previous return still encodes the old command.
                    # Refresh observations before the first action of each phase.
                    obs = env.get_observations()

                if (
                    stop_action_sensitivity_supported
                    and cycle_index == args_cli.motion_steps
                ):
                    actual_q = action_term._asset.data.joint_pos.torch[
                        :, action_term._joint_ids
                    ]
                    target_gap = torch.abs(actual_q - action_term._stop_targets)
                    sensitivity_limit = (
                        action_term._stop_residual_scale
                        * torch.tanh(torch.ones_like(action_term._stop_residual_scale))
                        + action_term._stop_rate_limit
                    )
                    action_sensitive = target_gap < sensitivity_limit
                    diagnostic_mask = metrics.intact
                    if diagnostic_mask.any():
                        counts = action_sensitive[diagnostic_mask].sum(dim=-1).to(torch.float64)
                        per_joint = (
                            100.0
                            * action_sensitive[diagnostic_mask]
                            .to(torch.float64)
                            .mean(dim=0)
                        )
                        print(
                            "  STOP首拍动作敏感关节 count[min/mean/max]="
                            f"[{int(counts.min().item())}/{counts.mean().item():.2f}/"
                            f"{int(counts.max().item())}] of {action_sensitive.shape[1]}；"
                            "逐关节敏感率%="
                            + ",".join(f"{value:.1f}" for value in per_joint.tolist()),
                            flush=True,
                        )

                if capture_latched_supported and not motion_phase:
                    captured_now = getattr(action_term, capture_latched_attribute).clone()
                    metrics.record_capture_latched(cycle_id, cycle_index, captured_now)
                    if cycle_index < args_cli.motion_steps + 50:
                        if capture_previous is not None:
                            capture_toggles += (
                                (captured_now != capture_previous) & metrics.intact
                            ).to(torch.int32)
                        capture_previous = captured_now.clone()

                actions = policy(obs)
                obs, _, dones, _ = env.step(actions)
                episode_steps += 1
                done = dones.bool()
                time_out = base_env.termination_manager.time_outs.bool()
                terminated = base_env.termination_manager.terminated.bool()

                term_matrix = torch.stack(
                    [
                        base_env.termination_manager.get_term(name).bool()
                        for name in termination_names
                    ],
                    dim=-1,
                )
                term_matrix &= done.unsqueeze(-1)

                metrics.update_step(
                    cycle_id,
                    cycle_index,
                    done,
                    time_out,
                    terminated,
                    term_matrix,
                    phase_targets,
                    motion_phase=motion_phase,
                    tail_phase=tail_phase,
                )
                metrics.update_event_stats(
                    done,
                    episode_steps,
                    term_matrix,
                    args_cli.episode_length,
                )

                done_ids = done.nonzero(as_tuple=False).squeeze(-1)
                if done_ids.numel() > 0:
                    episode_steps[done_ids] = 0
                if hasattr(policy, "reset"):
                    policy.reset(dones)

                # Auto-reset must have used the phase-aware replacement resampler.
                command_lock.assert_locked()
                completed_steps += 1
                if (
                    completed_steps % progress_interval == 0
                    or completed_steps == args_cli.num_steps
                ):
                    print(
                        f"  已完成 {completed_steps}/{args_cli.num_steps} 步，"
                        f"原始 done 事件 {int(metrics.done_events.sum().item())}",
                        flush=True,
                    )

            metrics.finish_cycle(cycle_id)
            if capture_latched_supported:
                toggles_64 = capture_toggles.to(torch.float64)
                print(
                    "  STOP前50拍 capture toggle[min/mean/max]="
                    f"[{int(capture_toggles.min().item())}/{toggles_64.mean().item():.2f}/"
                    f"{int(capture_toggles.max().item())}]；>2次环境="
                    f"{int(torch.count_nonzero(capture_toggles > 2).item())}/"
                    f"{args_cli.num_envs}",
                    flush=True,
                )
            else:
                print(
                    "  STOP capture toggle: unavailable "
                    "(任务 action term 未暴露 capture_latched latch tensor)",
                    flush=True,
                )

    print("\n=== 评估信息 ===")
    print(f"task: {args_cli.task}")
    print(f"checkpoint: {checkpoint_path}")
    print(f"并行环境数: {args_cli.num_envs}, 总步数: {args_cli.num_steps}")
    print(
        f"独立同步周期数: {num_cycles}; motion={args_cli.motion_steps}, "
        f"stop={args_cli.stop_steps}, stop_tail={args_cli.stop_tail_steps}; "
        f"motion_command_scale={args_cli.motion_command_scale:g}"
    )
    print(
        "停止控制: "
        f"linear_deceleration={args_cli.linear_deceleration:g}, "
        f"yaw_deceleration={args_cli.yaw_deceleration:g}, "
        f"contact_aligned_zero={args_cli.contact_aligned_zero}"
    )
    print(
        "验收阈值: "
        f"motion_planar_rmse<={args_cli.max_motion_planar_rmse:g}, "
        f"motion_yaw_rmse<={args_cli.max_motion_yaw_rmse:g}, "
        f"tail_planar_rms<={args_cli.max_tail_planar_rms:g}, "
        f"tail_yaw_rms<={args_cli.max_tail_yaw_rms:g}"
    )
    metrics.report()


def main() -> None:
    if args_cli.num_envs < len(MOTION_SCENARIOS):
        raise ValueError(f"num_envs 至少为 {len(MOTION_SCENARIOS)}，以覆盖全部非零场景")
    if args_cli.num_steps <= 0 or args_cli.episode_length <= 0:
        raise ValueError("num_steps and episode_length must be positive")
    if args_cli.motion_steps <= 0 or args_cli.stop_steps <= 0:
        raise ValueError("motion_steps and stop_steps must be positive")
    if not math.isfinite(args_cli.motion_command_scale) or not (
        0.0 < args_cli.motion_command_scale <= 1.0
    ):
        raise ValueError("motion-command-scale must be finite and in (0, 1]")
    quality_thresholds = (
        args_cli.max_motion_planar_rmse,
        args_cli.max_motion_yaw_rmse,
        args_cli.max_tail_planar_rms,
        args_cli.max_tail_yaw_rms,
    )
    if any(not math.isfinite(value) or value <= 0.0 for value in quality_thresholds):
        raise ValueError("motion/tail quality thresholds must be finite and positive")
    if not 0 < args_cli.stop_tail_steps <= args_cli.stop_steps:
        raise ValueError("stop_tail_steps must be in [1, stop_steps]")
    if args_cli.zero_command_target_leak is not None and not (
        0.0 <= args_cli.zero_command_target_leak <= 1.0
    ):
        raise ValueError("zero_command_target_leak must be in [0, 1]")
    if args_cli.zero_command_anchor_leak is not None and not (
        0.0 <= args_cli.zero_command_anchor_leak <= 1.0
    ):
        raise ValueError("zero_command_anchor_leak must be in [0, 1]")
    if args_cli.linear_deceleration < 0.0 or args_cli.yaw_deceleration < 0.0:
        raise ValueError("linear-deceleration and yaw-deceleration must be non-negative")
    if (args_cli.linear_deceleration == 0.0) != (args_cli.yaw_deceleration == 0.0):
        raise ValueError(
            "linear-deceleration and yaw-deceleration must both be zero or both positive"
        )
    if args_cli.contact_aligned_zero and args_cli.linear_deceleration == 0.0:
        raise ValueError("--contact-aligned-zero requires positive deceleration limits")
    if args_cli.contact_confirm_steps <= 0:
        raise ValueError("contact-confirm-steps must be positive")
    if not 0.0 < args_cli.minimum_command_scale < 1.0:
        raise ValueError("minimum-command-scale must be in (0, 1)")
    if (
        args_cli.zero_release_planar_threshold < 0.0
        or args_cli.zero_release_yaw_threshold < 0.0
        or args_cli.zero_release_actual_planar_speed < 0.0
        or args_cli.zero_release_actual_yaw_speed < 0.0
    ):
        raise ValueError("zero-release thresholds must be non-negative")
    cycle_steps = args_cli.motion_steps + args_cli.stop_steps
    if cycle_steps != args_cli.episode_length:
        raise ValueError(
            "为使满回合与过渡周期可比，motion_steps + stop_steps 必须等于 episode_length: "
            f"{args_cli.motion_steps} + {args_cli.stop_steps} != {args_cli.episode_length}"
        )
    if args_cli.num_steps % cycle_steps != 0:
        raise ValueError(
            "num_steps 必须由完整独立过渡周期组成: "
            f"{args_cli.num_steps} % {cycle_steps} != 0"
        )

    checkpoint_path = args_cli.checkpoint_path.expanduser().resolve()
    if not checkpoint_path.is_file():
        raise FileNotFoundError(f"checkpoint does not exist: {checkpoint_path}")

    print("加载 Isaac Lab 环境和 RSL-RL 策略...", flush=True)
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs)
    env_cfg.seed = args_cli.seed
    if args_cli.zero_command_target_leak is not None:
        action_cfg = env_cfg.actions.joint_pos
        if not hasattr(action_cfg, "zero_command_target_leak"):
            raise AttributeError(
                f"task {args_cli.task} action does not support zero_command_target_leak"
            )
        action_cfg.zero_command_target_leak = args_cli.zero_command_target_leak
        print(
            "诊断覆盖 zero_command_target_leak="
            f"{args_cli.zero_command_target_leak:g}",
            flush=True,
        )
    if args_cli.zero_command_anchor_leak is not None:
        action_cfg = env_cfg.actions.joint_pos
        if not hasattr(action_cfg, "zero_command_anchor_leak"):
            raise AttributeError(
                f"task {args_cli.task} action does not support zero_command_anchor_leak"
            )
        action_cfg.zero_command_anchor_leak = args_cli.zero_command_anchor_leak
        print(
            "诊断覆盖 zero_command_anchor_leak="
            f"{args_cli.zero_command_anchor_leak:g}",
            flush=True,
        )
    agent_cfg = load_cfg_from_registry(args_cli.task, "rsl_rl_cfg_entry_point")
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, metadata.version("rsl-rl-lib"))
    agent_cfg.device = args_cli.device
    agent_cfg.seed = args_cli.seed
    _validate_run_snapshot(checkpoint_path, env_cfg, agent_cfg)

    # Validate the immutable training snapshot first, then allow the evaluator
    # to use its explicitly requested horizon.  Transition-focused training
    # tasks may deliberately use short episodes to increase braking samples,
    # while acceptance still needs a long stationary tail.  This changes only
    # the evaluation time-out and leaves policy/action/reward provenance intact.
    control_dt = float(env_cfg.sim.dt) * int(env_cfg.decimation)
    requested_episode_length_s = args_cli.episode_length * control_dt
    if not math.isclose(
        float(env_cfg.episode_length_s),
        requested_episode_length_s,
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        print(
            "评估时延长 episode horizon: "
            f"training={env_cfg.episode_length_s:g}s -> "
            f"evaluation={requested_episode_length_s:g}s "
            f"({args_cli.episode_length} control steps)",
            flush=True,
        )
        env_cfg.episode_length_s = requested_episode_length_s

    raw_env = gym.make(args_cli.task, cfg=env_cfg)
    try:
        env = RslRlVecEnvWrapper(raw_env, clip_actions=agent_cfg.clip_actions)
    except Exception:
        raw_env.close()
        raise
    try:
        _evaluate_checkpoint(env, agent_cfg, checkpoint_path)
    finally:
        env.close()


if __name__ == "__main__":
    exit_code = 0
    try:
        main()
    except Exception:
        traceback.print_exc()
        exit_code = 1
    finally:
        simulation_app.close(exit_code=exit_code)
    if exit_code:
        raise SystemExit(exit_code)
