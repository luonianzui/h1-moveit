#!/usr/bin/env python3
"""Source and fixture tests for the read-only 8254 snapshot cache pair."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
MONITOR = PACKAGE / "scripts" / "h1_ros_snapshot_monitor_8254.py"
READER = PACKAGE / "scripts" / "h1_ros_snapshot_cache_8254.sh"
MONITOR_SOURCE = MONITOR.read_text(encoding="utf-8")
READER_SOURCE = READER.read_text(encoding="utf-8")


class SnapshotCache8254Test(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(prefix="h1_snapshot_8254_test.")
        self.cache_dir = Path(self.temporary.name)
        self.cache_dir.chmod(0o700)
        self.pid = os.getpid()

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def write_secure(self, name: str, content: str) -> Path:
        path = self.cache_dir / name
        if path.exists():
            path.chmod(0o600)
        path.write_text(content, encoding="ascii")
        path.chmod(0o400)
        return path

    def populate(self, mode: str, rows: list[str], *, age_seconds: float = 0.0) -> None:
        stamp = time.monotonic_ns() - int(age_seconds * 1_000_000_000.0)
        self.write_secure(
            "ready",
            f"H1_ROS_SNAPSHOT_MONITOR_8254 v1 READY {self.pid} {stamp}\n",
        )
        self.write_secure(
            f"{mode}.cache",
            "\n".join(
                (
                    f"H1_ROS_SNAPSHOT_CACHE_8254 v1 {mode} {self.pid} {stamp} {len(rows)}",
                    *rows,
                    "",
                )
            ),
        )

    def run_reader(self, mode: str, timeout: str = "0.1") -> subprocess.CompletedProcess[str]:
        environment = os.environ.copy()
        environment["H1_ROS_SNAPSHOT_CACHE_DIR"] = str(self.cache_dir)
        environment["H1_ROS_SNAPSHOT_CACHE_MAX_AGE"] = "1.0"
        return subprocess.run(
            ["bash", str(READER), mode, "--timeout", timeout],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=environment,
        )

    def test_sources_parse_and_monitor_has_no_application_publisher(self) -> None:
        subprocess.run(["bash", "-n", str(READER)], check=True)
        compile(MONITOR_SOURCE, str(MONITOR), "exec")
        self.assertIn('ROBOT_TOPIC: Final = "/joint_states"', MONITOR_SOURCE)
        self.assertIn('INCUBATOR_TOPIC: Final = "/incubator/joint_states"', MONITOR_SOURCE)
        self.assertEqual(MONITOR_SOURCE.count("self.create_subscription("), 2)
        self.assertNotIn("create_publisher(", MONITOR_SOURCE)
        self.assertNotIn(".publish(", MONITOR_SOURCE)
        self.assertIn("enable_rosout=False", MONITOR_SOURCE)
        self.assertIn("self.destroy_publisher(parameter_publisher)", MONITOR_SOURCE)
        self.assertIn("time.monotonic_ns()", MONITOR_SOURCE)
        self.assertIn("os.replace(", MONITOR_SOURCE)
        self.assertIn("os.fchmod(file_fd, 0o400)", MONITOR_SOURCE)

    def test_robot_and_incubator_rows_match_legacy_interface(self) -> None:
        robot_rows = ["left_elbow_joint 0.59999999999999998", "torso_joint 0"]
        self.populate("robot", robot_rows)
        result = self.run_reader("robot")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, "\n".join((*robot_rows, "")))
        self.assertEqual(result.stderr, "")

        incubator_rows = [
            "door_hinge_joint 1.0471975511965976 0",
            "handle_latch_joint 1.5707963267948966 -0.0001",
        ]
        self.populate("incubator", incubator_rows)
        result = self.run_reader("incubator")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout, "\n".join((*incubator_rows, "")))

    def test_stale_or_wrong_monitor_cache_fails_without_rows(self) -> None:
        self.populate("robot", ["joint_a 0"], age_seconds=2.0)
        stale = self.run_reader("robot")
        self.assertNotEqual(stale.returncode, 0)
        self.assertEqual(stale.stdout, "")
        self.assertIn("timed out", stale.stderr)

        self.populate("robot", ["joint_a 0"])
        cache = self.cache_dir / "robot.cache"
        cache.chmod(0o600)
        text = cache.read_text(encoding="ascii").replace(
            f"robot {self.pid}", f"robot {self.pid + 1}"
        )
        cache.write_text(text, encoding="ascii")
        cache.chmod(0o400)
        mismatch = self.run_reader("robot")
        self.assertNotEqual(mismatch.returncode, 0)
        self.assertEqual(mismatch.stdout, "")

    def test_mode_symlink_and_hardlink_are_rejected(self) -> None:
        self.populate("robot", ["joint_a 0"])
        cache = self.cache_dir / "robot.cache"
        cache.chmod(0o600)
        cache.chmod(0o644)
        permissive = self.run_reader("robot")
        self.assertNotEqual(permissive.returncode, 0)
        self.assertEqual(permissive.stdout, "")
        self.assertIn("mode 0400", permissive.stderr)

        cache.chmod(0o400)
        os.link(cache, self.cache_dir / "robot.second_link")
        hardlinked = self.run_reader("robot")
        self.assertNotEqual(hardlinked.returncode, 0)
        self.assertEqual(hardlinked.stdout, "")
        self.assertIn("link-count 1", hardlinked.stderr)

    def test_duplicate_nonfinite_and_wrong_cardinality_rows_are_rejected(self) -> None:
        for rows in (
            ["joint_a 0", "joint_a 1"],
            ["joint_a nan"],
            ["joint_a 0 1"],
        ):
            self.populate("robot", rows)
            result = self.run_reader("robot")
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(result.stdout, "")
            self.assertIn("joint rows are malformed", result.stderr)


if __name__ == "__main__":
    unittest.main()
