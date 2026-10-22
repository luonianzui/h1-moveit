"""Fail-closed velocity limiting for the future Nav2-to-V12 adapter."""

from dataclasses import dataclass
import math

import numpy as np

from .locomotion_contract import EXPECTED_GATES


@dataclass(frozen=True)
class VelocityLimits:
    """Velocity, acceleration, deadband, and timeout limits."""

    minimum: np.ndarray
    maximum: np.ndarray
    acceleration: np.ndarray
    deadband: np.ndarray
    timeout: float
    control_period: float

    @classmethod
    def from_contract(cls, contract):
        """Create limits from a validated locomotion contract."""
        control = contract['control']
        safety = control['safety_limits']
        return cls(
            minimum=np.asarray([
                safety['vx'][0], safety['vy'][0], safety['yaw_rate'][0],
            ], dtype=np.float64),
            maximum=np.asarray([
                safety['vx'][1], safety['vy'][1], safety['yaw_rate'][1],
            ], dtype=np.float64),
            acceleration=np.asarray([
                safety['linear_acceleration'],
                safety['linear_acceleration'],
                safety['angular_acceleration'],
            ], dtype=np.float64),
            deadband=np.asarray(
                control['command_deadband'], dtype=np.float64),
            timeout=float(control['command_timeout']),
            control_period=float(control['control_period']),
        )


class VelocityGate:
    """Require every gate before forwarding a rate-limited command."""

    def __init__(self, limits):
        self.limits = limits
        self.output = np.zeros(3, dtype=np.float64)
        self.last_update_time = None
        self.last_input_time = None
        self.last_observed_time = None

    def _validate_time(self, now):
        now = float(now)
        if not math.isfinite(now):
            raise ValueError('command time must be finite')
        if self.last_observed_time is not None and now < self.last_observed_time:
            raise ValueError('command time must be monotonic')
        self.last_observed_time = now
        return now

    def _stop(self, now):
        self.output.fill(0.0)
        self.last_update_time = now
        return self.output.copy()

    def apply(self, command, now, gates):
        """Filter one command and return exact zero unless every gate passes."""
        now = self._validate_time(now)
        command = np.asarray(command, dtype=np.float64).reshape(-1)
        if command.size != 3 or not np.isfinite(command).all():
            raise ValueError('velocity command must contain three finite values')
        self.last_input_time = now
        if any(gates.get(name) is not True for name in EXPECTED_GATES):
            return self._stop(now)
        if np.less_equal(np.abs(command), self.limits.deadband).all():
            return self._stop(now)

        target = np.clip(
            command, self.limits.minimum, self.limits.maximum)
        if self.last_update_time is None:
            elapsed = self.limits.control_period
        else:
            elapsed = now - self.last_update_time
            if elapsed > self.limits.timeout:
                self.output.fill(0.0)
                elapsed = self.limits.control_period
        maximum_delta = self.limits.acceleration * elapsed
        delta = np.clip(target - self.output, -maximum_delta, maximum_delta)
        self.output += delta
        self.last_update_time = now
        return self.output.copy()

    def watchdog(self, now):
        """Return exact zero when the input command is stale."""
        now = self._validate_time(now)
        if (
                self.last_input_time is None
                or now - self.last_input_time > self.limits.timeout):
            return self._stop(now)
        return self.output.copy()
