"""Cold-start arming state for the Gazebo V12 locomotion adapter."""

import numpy as np


class MotionArming:
    """Hold the training pose until a command reaches the trained onset."""

    def __init__(self, onset_threshold=(0.0, 0.0, 0.0)):
        """Start disarmed in the default-pose standby state."""
        self.onset_threshold = np.asarray(
            onset_threshold, dtype=np.float32).reshape(-1)
        if self.onset_threshold.size != 3 or not np.isfinite(
                self.onset_threshold).all():
            raise ValueError('onset threshold must contain three finite values')
        if np.any(self.onset_threshold < 0.0):
            raise ValueError('onset threshold must be non-negative')
        if np.any(self.onset_threshold == 0.0) and not np.all(
                self.onset_threshold == 0.0):
            raise ValueError('onset threshold must be all zero or all positive')
        self.motion_started = False

    def standby(self, command):
        """Return whether cold-start standby must remain active."""
        if not self.motion_started:
            self.motion_started = self.reaches_onset(command)
        return not self.motion_started

    def reaches_onset(self, command):
        """Return whether a command reaches the trained motion onset."""
        values = np.asarray(command, dtype=np.float32).reshape(-1)
        if values.size != 3 or not np.isfinite(values).all():
            raise ValueError('command must contain three finite values')
        if np.all(self.onset_threshold == 0.0):
            return bool(np.any(values != 0.0))
        scaled = values / self.onset_threshold
        return bool(np.linalg.norm(scaled) >= 1.0)
