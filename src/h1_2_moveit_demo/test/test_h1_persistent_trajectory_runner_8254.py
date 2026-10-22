import hashlib
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest


RUNNER = (
    Path(__file__).parents[1]
    / "scripts"
    / "h1_persistent_trajectory_runner_8254.py"
)
SPEC = importlib.util.spec_from_file_location("h1_persistent_runner_test", RUNNER)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class FakeAssetModule:
    def __init__(self, payload, digest):
        self.payload = payload
        self.digest = digest
        self.load_calls = 0

    def load_asset(self, _path):
        self.load_calls += 1
        return self.payload, [1_000_000_000], self.digest

    @staticmethod
    def _canonical_bytes(payload):
        return json.dumps(
            payload,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")

    @staticmethod
    def _parse_message_time_scales(_text):
        return {0: 0.5}

    @staticmethod
    def _selected_message_time_scales(parsed, _start, _end, _count):
        return parsed

    @staticmethod
    def _retimed_message_points(_message, _scale, _strict, _context):
        return None

    @staticmethod
    def _replay_overlap_groups(_messages, _durations, _start, _end, *_args):
        return (({"index": 0},),)

    @staticmethod
    def _trajectory_message(_message, _scale, _strict, _context):
        return object()


def make_payload():
    return {
        "schema": "h1_joint_trajectory_asset/v1",
        "stage": "door",
        "controller_topics": list(MODULE.CONTROLLER_TOPICS),
        "clock": {
            "wall_offset_unit": "nanoseconds",
            "time_from_start_unit": "sec_nanosec",
        },
        "messages": [{
            "sequence": 0,
            "topic": MODULE.CONTROLLER_TOPICS[0],
            "wall_offset_ns": 0,
            "joint_names": ["joint"],
            "points": [{
                "positions": [0.0],
                "velocities": [],
                "accelerations": [],
                "effort": [],
                "time_from_start": {"sec": 1, "nanosec": 0},
            }],
        }],
    }


class PersistentRunnerSourceTest(unittest.TestCase):
    def _synthetic_cache(self, directory):
        asset = directory / "assets" / "door.json"
        asset.parent.mkdir()
        raw = b'{"sealed":"test"}\n'
        asset.write_bytes(raw)
        os.chmod(asset, 0o400)
        payload = make_payload()
        payload_digest = hashlib.sha256(
            FakeAssetModule._canonical_bytes(payload)
        ).hexdigest()
        registration = MODULE.RegisteredAsset(
            "assets/door.json",
            "door",
            1,
            payload_digest,
            hashlib.sha256(raw).hexdigest(),
        )
        fake = FakeAssetModule(payload, payload_digest)
        cache = MODULE._preload_registered_assets(
            fake, directory, (registration,)
        )
        request = {
            "asset": asset,
            "stage": "door",
            "start": 0,
            "end": 1,
            "scales": "0:0.5",
            "expected_digest": payload_digest,
        }
        return asset, fake, cache, request

    def test_startup_cache_is_immutable_and_replay_does_not_reparse_json(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary).resolve()
            asset, fake, cache, request = self._synthetic_cache(directory)
            self.assertEqual(fake.load_calls, 1)
            cached = MODULE._cached_asset_for_request(request, cache)
            with self.assertRaises(TypeError):
                cached.payload["stage"] = "cube"
            with self.assertRaises(TypeError):
                cached.messages[0]["topic"] = "bad"
            messages, outgoing, groups, digest = MODULE._prepare_replay(
                fake,
                request,
                {MODULE.CONTROLLER_TOPICS[0]: object()},
                cache,
            )
            self.assertIsNot(messages, cached.messages)
            self.assertEqual(messages[0]["topic"], cached.messages[0]["topic"])
            self.assertEqual(set(outgoing), {0})
            self.assertEqual(groups, (({"index": 0},),))
            self.assertEqual(digest, request["expected_digest"])
            self.assertEqual(fake.load_calls, 1)
            self.assertEqual(asset.stat().st_mode & 0o777, 0o400)

    def test_request_revalidation_rejects_any_file_or_metadata_change(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary).resolve()
            asset, _fake, cache, request = self._synthetic_cache(directory)
            os.chmod(asset, 0o600)
            with self.assertRaises(MODULE.RunnerError):
                MODULE._cached_asset_for_request(request, cache)

        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary).resolve()
            asset, _fake, cache, request = self._synthetic_cache(directory)
            os.chmod(asset, 0o600)
            asset.write_bytes(b'{"sealed":"changed"}\n')
            os.chmod(asset, 0o400)
            with self.assertRaises(MODULE.RunnerError):
                MODULE._cached_asset_for_request(request, cache)

    def test_request_must_match_registered_path_stage_and_payload_digest(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary).resolve()
            _asset, _fake, cache, request = self._synthetic_cache(directory)
            for field, value in (
                ("asset", directory / "not-registered.json"),
                ("stage", "cube"),
                ("expected_digest", "2" * 64),
            ):
                changed = dict(request)
                changed[field] = value
                with self.subTest(field=field), self.assertRaises(MODULE.RunnerError):
                    MODULE._cached_asset_for_request(changed, cache)

    def test_production_registry_is_complete_and_has_unique_bindings(self):
        registrations = MODULE.REGISTERED_ASSETS
        self.assertEqual(len(registrations), 7)
        self.assertEqual(len({item.relative_path for item in registrations}), 7)
        self.assertEqual(
            {item.stage for item in registrations},
            {"handle", "left_clear", "door", "cube"},
        )
        for item in registrations:
            self.assertEqual(len(item.payload_sha256), 64)
            self.assertEqual(len(item.file_sha256), 64)

    def test_only_sealed_reverse_handle_split_ranges_are_authorized(self):
        digest = next(iter(MODULE.SEALED_HANDLE_SEGMENT_RANGES))
        for start, end in ((0, 1), (1, 5), (0, 5)):
            with self.subTest(start=start, end=end):
                MODULE._require_authorized_range("handle", start, end, 5, digest)
        for start, end, wrong_digest, count in (
            (0, 2, digest, 5),
            (1, 4, digest, 5),
            (0, 1, "a" * 64, 5),
            (1, 5, digest, 6),
        ):
            with self.subTest(start=start, end=end, digest=wrong_digest, count=count):
                with self.assertRaises(MODULE.RunnerError):
                    MODULE._require_authorized_range(
                        "handle", start, end, count, wrong_digest
                    )

    def test_simulation_tolerances_are_request_scoped_and_bounded(self):
        source = RUNNER.read_text(encoding="utf-8")
        self.assertIn("not (0.0 < value <= 0.012)", source)
        self.assertIn(
            'stage == "cube" and (start, end) == (2, 4)', source
        )
        relaxed = MODULE._parse_request_line(
            "run\t1\t/absolute/asset.json\tcube\t2\t4\t2:0.6,3:0.6\t"
            + "a" * 64
            + "\t0.012\t0.012"
        )
        self.assertEqual(relaxed["endpoint_tolerance"], 0.012)
        with self.assertRaises(MODULE.RunnerError):
            MODULE._parse_request_line(
                "run\t1\t/absolute/asset.json\tdoor\t0\t1\t0:0.1\t"
                + "a" * 64
                + "\t0.012\t0.012"
            )

    def test_run_request_is_strict_and_complete(self):
        request = MODULE._parse_request_line(
            "run\t7\t/absolute/asset.json\tdoor\t6\t13\t"
            "6:0.1,7:0.1,8:0.1,9:0.1,10:0.1,11:0.1,12:0.1\t"
            + "a" * 64
            + "\t0.003\t0.003"
        )
        self.assertEqual(request["operation"], "run")
        self.assertEqual(request["request_id"], 7)
        self.assertEqual(request["asset"], Path("/absolute/asset.json"))
        self.assertEqual((request["stage"], request["start"], request["end"]), ("door", 6, 13))

    def test_stop_request_has_no_payload(self):
        self.assertEqual(
            MODULE._parse_request_line("stop\t8" + "\t" * 8),
            {"operation": "stop", "request_id": 8},
        )
        with self.assertRaises(MODULE.RunnerError):
            MODULE._parse_request_line("stop\t8\t/x" + "\t" * 7)

    def test_malformed_requests_fail_closed(self):
        bad = (
            "run\t0\t/a\tdoor\t0\t1\t0:1\t" + "a" * 64 + "\t0.003\t0.003",
            "run\t1\trelative\tdoor\t0\t1\t0:1\t" + "a" * 64 + "\t0.003\t0.003",
            "run\t1\t/a\tbad\t0\t1\t0:1\t" + "a" * 64 + "\t0.003\t0.003",
            "run\t1\t/a\tdoor\t1\t1\t1:1\t" + "a" * 64 + "\t0.003\t0.003",
            "run\t1\t/a\tdoor\tx\t1\t0:1\t" + "a" * 64 + "\t0.003\t0.003",
            "run\t1\t/a\tdoor\t0\t1\t" + "a" * 64 + "\t0.003\t0.003",
            "run\t1\t/a\tdoor\t0\t1\t0:1\t" + "A" * 64 + "\t0.003\t0.003",
            "run\t1\t/a\tdoor\t0\t1\t0:1\t" + "a" * 64 + "\t0.003\t0.003\nextra",
        )
        for row in bad:
            with self.subTest(row=row), self.assertRaises(MODULE.RunnerError):
                MODULE._parse_request_line(row)

    def test_private_directory_and_exclusive_file_contract(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            os.chmod(directory, 0o700)
            MODULE._private_directory(directory)
            target = directory / "response"
            MODULE._exclusive_text(target, "PASS\n", 0o400)
            metadata = target.stat()
            self.assertEqual(metadata.st_uid, os.geteuid())
            self.assertEqual(metadata.st_mode & 0o777, 0o400)
            self.assertEqual(metadata.st_nlink, 1)
            with self.assertRaises(FileExistsError):
                MODULE._exclusive_text(target, "overwrite\n", 0o400)


if __name__ == "__main__":
    unittest.main()
