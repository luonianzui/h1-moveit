#!/usr/bin/env python3
"""Seal and independently verify the Candidate-3 fast-90 timing claim.

This sidecar deliberately proves only timing.  It accepts a completed, PASS
full-flow live certificate as its semantic authority, then binds the two
controller stdout logs, the supervisor marker transcript and process snapshot,
and the sealed sqlite3 rosbag.  Wall time comes from unique monotonic markers
printed by the controller flow.  A conservative simulation-time upper bound is
formed from the last ``/clock`` sample stored no later than SUPPORT_PRELOAD and
the first ``/clock`` sample stored no earlier than FLOW_MIDDLE_SUFFIX_PASS.

The output is canonical JSON, mode 0400, and is published without replacement.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import sqlite3
import stat
import struct
import tempfile
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Mapping, Sequence, Tuple

try:
    import yaml
except ImportError:  # pragma: no cover - the ROS 2 runtime ships PyYAML.
    yaml = None


SCHEMA = "h1_candidate3_fast90_timing_certificate/831-v1"
VERIFIER_VERSION = "h1_candidate3_fast90_certificate_831/1"
MAIN_CERTIFICATE_SCHEMA = "h1_candidate3_full_flow_live_certificate/829-v1"
MARKER_SCHEMA = "h1_candidate3_live_marker/829-v1"
PROC_SCHEMA = "h1_candidate3_live_proc_snapshot/829-v1"
MARKER_TOPIC = "/h1_candidate3/live_829/markers"
CLOCK_TOPIC = "/clock"
MAX_DURATION_NS = 90_000_000_000
MAX_JSON_BYTES = 16 * 1024 * 1024
MAX_LOG_BYTES = 16 * 1024 * 1024
MAX_METADATA_BYTES = 2 * 1024 * 1024
MAX_DB_BYTES = 16 * 1024 * 1024 * 1024
SHA256_RE = re.compile(r"[0-9a-f]{64}")
RUN_ID_RE = SHA256_RE
START_RE = re.compile(
    r"^CANDIDATE3_FAST_831_MOTION_WINDOW=START monotonic_ns=([1-9][0-9]*)$",
    re.MULTILINE,
)
END_RE = re.compile(
    r"^CANDIDATE3_FAST_831_MOTION_WINDOW=END monotonic_ns=([1-9][0-9]*)$",
    re.MULTILINE,
)
SUPPORT_PRELOAD_LINE = (
    "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=preload "
    "door=detached cube=detached"
)
FLOW_PASS_LINE = "CANDIDATE3_FLOW_828_MIDDLE_SUFFIX=PASS"


class Fast90CertificateError(RuntimeError):
    """A stable, machine-readable timing-certificate failure."""


@dataclass(frozen=True)
class FileEvidence:
    path: Path
    raw: bytes
    sha256: str
    size_bytes: int
    mode: str


def _require(condition: bool, status: str) -> None:
    if not condition:
        raise Fast90CertificateError(status)


def _exact(value: Any, keys: Iterable[str], status: str) -> Mapping[str, Any]:
    _require(type(value) is dict, f"{status}_TYPE")
    _require(frozenset(value) == frozenset(keys), f"{status}_KEYS")
    return value


def _integer(value: Any, status: str, minimum: int = 0) -> int:
    _require(type(value) is int and value >= minimum, status)
    return value


def _sha(value: Any, status: str) -> str:
    _require(type(value) is str and SHA256_RE.fullmatch(value) is not None,
             status)
    return value


def _run_id(value: Any, status: str) -> str:
    _require(type(value) is str and RUN_ID_RE.fullmatch(value) is not None,
             status)
    return value


def _reject_duplicate_keys(
        pairs: Sequence[Tuple[str, Any]]) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise Fast90CertificateError(f"DUPLICATE_JSON_KEY:{key}")
        result[key] = value
    return result


def _reject_constant(token: str) -> None:
    raise Fast90CertificateError(f"NONFINITE_JSON:{token}")


def _canonical_bytes(value: Any) -> bytes:
    try:
        return (json.dumps(
            value, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
            allow_nan=False).encode("utf-8") + b"\n")
    except (TypeError, ValueError) as error:
        raise Fast90CertificateError(f"CANONICAL_JSON:{error}") from error


def _canonical_hash(value: Any) -> str:
    return hashlib.sha256(_canonical_bytes(value)[:-1]).hexdigest()


def _canonical_absolute_path(value: Any, status: str) -> Path:
    _require(type(value) is str, f"{status}_TYPE")
    path = Path(value)
    _require(path.is_absolute() and os.path.normpath(value) == value,
             status)
    return path


def _read_sealed_file(
    path: Path,
    *,
    expected_sha256: str | None = None,
    maximum_bytes: int,
    status: str,
) -> FileEvidence:
    _require(path.is_absolute() and os.path.normpath(str(path)) == str(path),
             f"{status}_PATH")
    try:
        metadata = path.lstat()
        resolved = path.resolve(strict=True)
    except OSError as error:
        raise Fast90CertificateError(f"{status}_STAT:{error}") from error
    _require(stat.S_ISREG(metadata.st_mode) and not path.is_symlink() and
             resolved == path, f"{status}_IDENTITY")
    _require(metadata.st_uid == os.geteuid() and metadata.st_nlink == 1 and
             stat.S_IMODE(metadata.st_mode) == 0o400,
             f"{status}_SEALED")
    _require(0 < metadata.st_size <= maximum_bytes, f"{status}_SIZE")
    try:
        raw = path.read_bytes()
    except OSError as error:
        raise Fast90CertificateError(f"{status}_READ:{error}") from error
    _require(len(raw) == metadata.st_size, f"{status}_SHORT_READ")
    digest = hashlib.sha256(raw).hexdigest()
    if expected_sha256 is not None:
        _require(digest == _sha(expected_sha256, f"{status}_EXPECTED_SHA256"),
                 f"{status}_SHA256")
    return FileEvidence(
        path=path, raw=raw, sha256=digest, size_bytes=len(raw), mode="0400")


def _binding(evidence: FileEvidence) -> Mapping[str, Any]:
    return {
        "mode": evidence.mode,
        "path": str(evidence.path),
        "sha256": evidence.sha256,
        "size_bytes": evidence.size_bytes,
    }


def _load_canonical_json(evidence: FileEvidence, status: str) -> Mapping[str, Any]:
    try:
        value = json.loads(
            evidence.raw.decode("utf-8"),
            object_pairs_hook=_reject_duplicate_keys,
            parse_constant=_reject_constant,
        )
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise Fast90CertificateError(f"{status}_JSON:{error}") from error
    _require(type(value) is dict, f"{status}_ROOT")
    _require(evidence.raw == _canonical_bytes(value), f"{status}_CANONICAL")
    return value


def _validate_main_certificate(
    value: Mapping[str, Any],
    *,
    markers: FileEvidence,
    proc_snapshot: FileEvidence,
    rosbag: FileEvidence,
) -> str:
    _require(value.get("schema") == MAIN_CERTIFICATE_SCHEMA,
             "MAIN_CERTIFICATE_SCHEMA")
    run_id = _run_id(value.get("run_id"), "MAIN_CERTIFICATE_RUN_ID")
    _require(value.get("offline_only") is False and
             value.get("live_execution_passed") is True and
             value.get("valid") is True and value.get("status") == "PASS",
             "MAIN_CERTIFICATE_NOT_PASS")
    inputs = value.get("inputs")
    _require(type(inputs) is dict, "MAIN_CERTIFICATE_INPUTS")
    expected = {
        "markers": markers,
        "proc_snapshot": proc_snapshot,
        "rosbag": rosbag,
    }
    for identifier, evidence in expected.items():
        binding = inputs.get(identifier)
        _require(type(binding) is dict, f"MAIN_INPUT_{identifier}_TYPE")
        _require(binding == _binding(evidence),
                 f"MAIN_INPUT_{identifier}_BINDING")
    marker_summary = value.get("markers")
    _require(type(marker_summary) is dict and
             marker_summary.get("strict_order_passed") is True,
             "MAIN_CERTIFICATE_MARKERS")
    names = marker_summary.get("ordered_names")
    timing_names = (
        "SUPPORT_PRELOAD", "MOTION_WINDOW_START", "MOTION_WINDOW_END",
        "FLOW_MIDDLE_SUFFIX_PASS",
    )
    _require(type(names) is list and
             all(names.count(name) == 1 for name in timing_names) and
             [names.index(name) for name in timing_names] == sorted(
                 names.index(name) for name in timing_names),
             "MAIN_CERTIFICATE_TIMING_MARKERS")
    motion = marker_summary.get("motion_window")
    _require(type(motion) is dict and
             motion.get("maximum_ns") == MAX_DURATION_NS and
             motion.get("within_90_seconds") is True,
             "MAIN_CERTIFICATE_MOTION_WINDOW")
    printed_start = _integer(
        motion.get("printed_start_monotonic_ns"),
        "MAIN_CERTIFICATE_MOTION_START", 1)
    printed_end = _integer(
        motion.get("printed_end_monotonic_ns"),
        "MAIN_CERTIFICATE_MOTION_END", 1)
    printed_duration = _integer(
        motion.get("printed_duration_ns"),
        "MAIN_CERTIFICATE_MOTION_PRINTED_DURATION", 1)
    observed_duration = _integer(
        motion.get("observed_duration_ns"),
        "MAIN_CERTIFICATE_MOTION_OBSERVED_DURATION", 1)
    _require(printed_end - printed_start == printed_duration <=
             MAX_DURATION_NS and observed_duration <= MAX_DURATION_NS,
             "MAIN_CERTIFICATE_MOTION_DURATION")
    return run_id


def _load_markers(
    evidence: FileEvidence, run_id: str,
) -> Tuple[Mapping[str, Any], ...]:
    _require(evidence.raw.endswith(b"\n"), "MARKERS_FRAMING")
    rows = []
    previous_monotonic = -1
    previous_bag = -1
    seen_names = set()
    for index, raw_line in enumerate(evidence.raw.splitlines(keepends=True)):
        try:
            value = json.loads(
                raw_line.decode("utf-8"),
                object_pairs_hook=_reject_duplicate_keys,
                parse_constant=_reject_constant,
            )
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise Fast90CertificateError(
                f"MARKER_{index}_JSON:{error}") from error
        marker = _exact(value, (
            "schema", "run_id", "sequence", "monotonic_ns",
            "bag_timestamp_ns", "name", "process_role", "line",
        ), f"MARKER_{index}")
        _require(raw_line == _canonical_bytes(marker),
                 f"MARKER_{index}_CANONICAL")
        _require(marker["schema"] == MARKER_SCHEMA and
                 marker["run_id"] == run_id and marker["sequence"] == index,
                 f"MARKER_{index}_IDENTITY")
        monotonic_ns = _integer(
            marker["monotonic_ns"], f"MARKER_{index}_MONOTONIC", 1)
        bag_ns = _integer(
            marker["bag_timestamp_ns"], f"MARKER_{index}_BAG_TIME", 1)
        _require(monotonic_ns > previous_monotonic and bag_ns > previous_bag,
                 f"MARKER_{index}_ORDER")
        _require(type(marker["name"]) is str and marker["name"] and
                 marker["name"] not in seen_names and
                 type(marker["process_role"]) is str and
                 type(marker["line"]) is str,
                 f"MARKER_{index}_FIELDS")
        previous_monotonic = monotonic_ns
        previous_bag = bag_ns
        seen_names.add(marker["name"])
        rows.append(marker)
    _require(rows, "MARKERS_EMPTY")
    return tuple(rows)


def _marker_by_name(
    markers: Sequence[Mapping[str, Any]], name: str, role: str,
) -> Mapping[str, Any]:
    rows = [marker for marker in markers if marker["name"] == name]
    _require(len(rows) == 1, f"MARKER_{name}_COUNT")
    _require(rows[0]["process_role"] == role, f"MARKER_{name}_ROLE")
    return rows[0]


def _validate_process_snapshot(
    value: Mapping[str, Any], run_id: str,
) -> Mapping[str, Mapping[str, int]]:
    _require(value.get("schema") == PROC_SCHEMA and
             value.get("run_id") == run_id and value.get("valid") is True and
             value.get("status") == "PASS", "PROC_IDENTITY")
    processes = value.get("processes")
    _require(type(processes) is list, "PROC_PROCESSES")
    result: Dict[str, Mapping[str, int]] = {}
    for role in ("prefix", "middle_suffix"):
        matches = [row for row in processes
                   if type(row) is dict and row.get("role") == role]
        _require(len(matches) == 1, f"PROC_{role}_COUNT")
        row = matches[0]
        _require(row.get("run_id") == run_id and row.get("exit_code") == 0,
                 f"PROC_{role}_RESULT")
        started = _integer(
            row.get("started_monotonic_ns"), f"PROC_{role}_START", 1)
        ended = _integer(
            row.get("ended_monotonic_ns"), f"PROC_{role}_END", 1)
        _require(started < ended, f"PROC_{role}_INTERVAL")
        result[role] = {
            "started_monotonic_ns": started,
            "ended_monotonic_ns": ended,
        }
    _require(result["prefix"]["ended_monotonic_ns"] <=
             result["middle_suffix"]["started_monotonic_ns"],
             "PROC_FLOW_ORDER")
    return result


def _decode_log(evidence: FileEvidence, status: str) -> str:
    try:
        text = evidence.raw.decode("ascii")
    except UnicodeDecodeError as error:
        raise Fast90CertificateError(f"{status}_ENCODING:{error}") from error
    _require(text.endswith("\n") and "\x00" not in text,
             f"{status}_FRAMING")
    return text


def _unique_motion_window(
    prefix_text: str,
    middle_text: str,
    process_intervals: Mapping[str, Mapping[str, int]],
    support_marker: Mapping[str, Any],
    motion_start_marker: Mapping[str, Any],
    motion_end_marker: Mapping[str, Any],
    flow_marker: Mapping[str, Any],
) -> Mapping[str, int]:
    combined = prefix_text + middle_text
    starts = START_RE.findall(combined)
    ends = END_RE.findall(combined)
    _require(len(starts) == 1 and len(ends) == 1,
             "MOTION_WINDOW_UNIQUE")
    _require(len(START_RE.findall(prefix_text)) == 1 and
             not START_RE.search(middle_text) and
             len(END_RE.findall(middle_text)) == 1 and
             not END_RE.search(prefix_text),
             "MOTION_WINDOW_LOG_ROLE")
    _require(prefix_text.count(SUPPORT_PRELOAD_LINE) == 1 and
             prefix_text.index(SUPPORT_PRELOAD_LINE) <
             START_RE.search(prefix_text).start(),
             "MOTION_START_AFTER_SUPPORT_PRELOAD")
    _require(middle_text.count(FLOW_PASS_LINE) == 1 and
             END_RE.search(middle_text).end() < middle_text.index(FLOW_PASS_LINE),
             "MOTION_END_BEFORE_FLOW_PASS")
    start_ns = int(starts[0])
    end_ns = int(ends[0])
    marker_start_match = START_RE.fullmatch(motion_start_marker["line"])
    marker_end_match = END_RE.fullmatch(motion_end_marker["line"])
    _require(marker_start_match is not None and marker_end_match is not None and
             int(marker_start_match.group(1)) == start_ns and
             int(marker_end_match.group(1)) == end_ns,
             "MOTION_WINDOW_MARKER_STDOUT_BINDING")
    prefix_interval = process_intervals["prefix"]
    middle_interval = process_intervals["middle_suffix"]
    _require(prefix_interval["started_monotonic_ns"] <= start_ns <=
             prefix_interval["ended_monotonic_ns"],
             "MOTION_START_PROCESS_WINDOW")
    _require(middle_interval["started_monotonic_ns"] <= end_ns <=
             middle_interval["ended_monotonic_ns"],
             "MOTION_END_PROCESS_WINDOW")
    _require(start_ns < end_ns and
             support_marker["sequence"] < motion_start_marker["sequence"] <
             motion_end_marker["sequence"] < flow_marker["sequence"] and
             motion_start_marker["monotonic_ns"] <
             motion_end_marker["monotonic_ns"] < flow_marker["monotonic_ns"],
             "MOTION_MONOTONIC_ORDER")
    # Marker reception can trail the producer's next stdout line slightly, so
    # SUPPORT_PRELOAD is ordered semantically by the sealed prefix transcript.
    _require(abs(start_ns - support_marker["monotonic_ns"]) <=
             10_000_000_000, "MOTION_START_MARKER_PROXIMITY")
    duration_ns = end_ns - start_ns
    _require(duration_ns <= MAX_DURATION_NS, "WALL_DURATION_OVER_90_SECONDS")
    return {
        "start_monotonic_ns": start_ns,
        "end_monotonic_ns": end_ns,
        "duration_ns": duration_ns,
    }


def _validate_metadata(
    evidence: FileEvidence, rosbag_path: Path,
) -> Mapping[str, int]:
    _require(yaml is not None, "PYYAML_UNAVAILABLE")
    try:
        value = yaml.safe_load(evidence.raw.decode("utf-8"))
    except (UnicodeDecodeError, yaml.YAMLError) as error:
        raise Fast90CertificateError(f"ROSBAG_METADATA_YAML:{error}") from error
    _require(type(value) is dict and
             set(value) == {"rosbag2_bagfile_information"},
             "ROSBAG_METADATA_ROOT")
    info = value["rosbag2_bagfile_information"]
    _require(type(info) is dict and info.get("version") == 9 and
             info.get("storage_identifier") == "sqlite3" and
             info.get("compression_format") == "" and
             info.get("compression_mode") == "",
             "ROSBAG_METADATA_FORMAT")
    _require(info.get("relative_file_paths") == [rosbag_path.name],
             "ROSBAG_METADATA_RELATIVE_PATH")
    files = info.get("files")
    _require(type(files) is list and len(files) == 1 and
             type(files[0]) is dict and files[0].get("path") == rosbag_path.name,
             "ROSBAG_METADATA_FILES")
    total = _integer(info.get("message_count"),
                     "ROSBAG_METADATA_MESSAGE_COUNT", 1)
    topics = info.get("topics_with_message_count")
    _require(type(topics) is list, "ROSBAG_METADATA_TOPICS")
    counts: Dict[str, int] = {}
    for index, row in enumerate(topics):
        _require(type(row) is dict and
                 set(row) == {"topic_metadata", "message_count"},
                 f"ROSBAG_METADATA_TOPIC_{index}")
        metadata = row["topic_metadata"]
        _require(type(metadata) is dict, f"ROSBAG_METADATA_TOPIC_{index}_META")
        name = metadata.get("name")
        _require(type(name) is str and name not in counts and
                 metadata.get("serialization_format") == "cdr",
                 f"ROSBAG_METADATA_TOPIC_{index}_IDENTITY")
        counts[name] = _integer(
            row["message_count"], f"ROSBAG_METADATA_TOPIC_{index}_COUNT")
    _require(sum(counts.values()) == total, "ROSBAG_METADATA_TOTAL")
    _require(CLOCK_TOPIC in counts and counts[CLOCK_TOPIC] > 1 and
             MARKER_TOPIC in counts and counts[MARKER_TOPIC] > 1,
             "ROSBAG_METADATA_REQUIRED_TOPICS")
    return {
        "message_count": total,
        "clock_message_count": counts[CLOCK_TOPIC],
        "marker_message_count": counts[MARKER_TOPIC],
    }


def _decode_clock_cdr(raw: bytes, status: str) -> int:
    _require(len(raw) == 12, f"{status}_SIZE")
    encapsulation = raw[:4]
    if encapsulation == b"\x00\x01\x00\x00":
        endian = "<"
    elif encapsulation == b"\x00\x00\x00\x00":
        endian = ">"
    else:
        raise Fast90CertificateError(f"{status}_ENCAPSULATION")
    seconds, nanoseconds = struct.unpack_from(f"{endian}iI", raw, 4)
    _require(seconds >= 0 and nanoseconds < 1_000_000_000,
             f"{status}_VALUE")
    return seconds * 1_000_000_000 + nanoseconds


def _analyze_rosbag(
    rosbag: FileEvidence,
    metadata_summary: Mapping[str, int],
    support_marker: Mapping[str, Any],
    flow_marker: Mapping[str, Any],
    marker_count: int,
) -> Mapping[str, Any]:
    for suffix in ("-wal", "-shm", "-journal"):
        sidecar = Path(str(rosbag.path) + suffix)
        _require(not sidecar.exists() and not sidecar.is_symlink(),
                 f"ROSBAG_UNSEALED_SIDECAR:{suffix}")
    uri = f"file:{rosbag.path}?mode=ro&immutable=1"
    try:
        connection = sqlite3.connect(uri, uri=True)
    except sqlite3.Error as error:
        raise Fast90CertificateError(f"ROSBAG_OPEN:{error}") from error
    try:
        integrity = connection.execute("PRAGMA integrity_check").fetchall()
        _require(integrity == [("ok",)], "ROSBAG_INTEGRITY")
        topic_rows = connection.execute(
            "SELECT id,name,type,serialization_format FROM topics "
            "WHERE name IN (?,?) ORDER BY name", (CLOCK_TOPIC, MARKER_TOPIC),
        ).fetchall()
        _require(len(topic_rows) == 2, "ROSBAG_REQUIRED_TOPIC_COUNT")
        topics = {row[1]: row for row in topic_rows}
        clock_topic = topics[CLOCK_TOPIC]
        marker_topic = topics[MARKER_TOPIC]
        _require(clock_topic[2:] == ("rosgraph_msgs/msg/Clock", "cdr") and
                 marker_topic[2:] == ("std_msgs/msg/String", "cdr"),
                 "ROSBAG_REQUIRED_TOPIC_TYPES")
        total_count = connection.execute(
            "SELECT COUNT(*) FROM messages").fetchone()[0]
        clock_count = connection.execute(
            "SELECT COUNT(*) FROM messages WHERE topic_id=?",
            (clock_topic[0],),
        ).fetchone()[0]
        stored_marker_count = connection.execute(
            "SELECT COUNT(*) FROM messages WHERE topic_id=?",
            (marker_topic[0],),
        ).fetchone()[0]
        _require(total_count == metadata_summary["message_count"] and
                 clock_count == metadata_summary["clock_message_count"] and
                 stored_marker_count ==
                 metadata_summary["marker_message_count"] == marker_count,
                 "ROSBAG_MESSAGE_COUNTS")
        previous_storage = -1
        previous_clock = -1
        maximum_step = 0
        for index, (storage_ns, raw) in enumerate(connection.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? "
            "ORDER BY timestamp,id", (clock_topic[0],),
        )):
            clock_ns = _decode_clock_cdr(raw, f"CLOCK_{index}")
            _require(storage_ns > previous_storage and clock_ns >= previous_clock,
                     f"CLOCK_{index}_ORDER")
            if previous_clock >= 0:
                maximum_step = max(maximum_step, clock_ns - previous_clock)
            previous_storage = storage_ns
            previous_clock = clock_ns
        lower = connection.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? AND "
            "timestamp<=? ORDER BY timestamp DESC,id DESC LIMIT 1",
            (clock_topic[0], support_marker["bag_timestamp_ns"]),
        ).fetchone()
        upper = connection.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? AND "
            "timestamp>=? ORDER BY timestamp ASC,id ASC LIMIT 1",
            (clock_topic[0], flow_marker["bag_timestamp_ns"]),
        ).fetchone()
        _require(lower is not None and upper is not None,
                 "ROSBAG_CLOCK_BRACKETS")
        lower_clock = _decode_clock_cdr(lower[1], "CLOCK_START_LOWER")
        upper_clock = _decode_clock_cdr(upper[1], "CLOCK_END_UPPER")
        _require(lower[0] <= support_marker["bag_timestamp_ns"] <
                 flow_marker["bag_timestamp_ns"] <= upper[0] and
                 lower_clock <= upper_clock,
                 "ROSBAG_CLOCK_BRACKET_ORDER")
        duration_ns = upper_clock - lower_clock
        _require(duration_ns <= MAX_DURATION_NS,
                 "SIM_DURATION_UPPER_BOUND_OVER_90_SECONDS")
        return {
            "clock_message_count": clock_count,
            "maximum_clock_step_ns": maximum_step,
            "start_lower": {
                "anchor_marker": "SUPPORT_PRELOAD",
                "anchor_storage_timestamp_ns":
                    support_marker["bag_timestamp_ns"],
                "clock_storage_timestamp_ns": lower[0],
                "clock_ns": lower_clock,
            },
            "end_upper": {
                "anchor_marker": "FLOW_MIDDLE_SUFFIX_PASS",
                "anchor_storage_timestamp_ns": flow_marker["bag_timestamp_ns"],
                "clock_storage_timestamp_ns": upper[0],
                "clock_ns": upper_clock,
            },
            "duration_upper_bound_ns": duration_ns,
        }
    except sqlite3.Error as error:
        raise Fast90CertificateError(f"ROSBAG_QUERY:{error}") from error
    finally:
        connection.close()


def _validate_binding(value: Any, status: str) -> Mapping[str, Any]:
    binding = _exact(value, ("mode", "path", "sha256", "size_bytes"), status)
    _require(binding["mode"] == "0400", f"{status}_MODE")
    _canonical_absolute_path(binding["path"], f"{status}_PATH")
    _sha(binding["sha256"], f"{status}_SHA256")
    _integer(binding["size_bytes"], f"{status}_SIZE", 1)
    return binding


def _build_value(
    *,
    main_certificate: FileEvidence,
    prefix_stdout: FileEvidence,
    middle_stdout: FileEvidence,
    markers_evidence: FileEvidence,
    proc_evidence: FileEvidence,
    metadata_evidence: FileEvidence,
    rosbag_evidence: FileEvidence,
) -> Mapping[str, Any]:
    main_value = _load_canonical_json(main_certificate, "MAIN_CERTIFICATE")
    run_id = _validate_main_certificate(
        main_value, markers=markers_evidence, proc_snapshot=proc_evidence,
        rosbag=rosbag_evidence)
    markers = _load_markers(markers_evidence, run_id)
    support_marker = _marker_by_name(markers, "SUPPORT_PRELOAD", "prefix")
    motion_start_marker = _marker_by_name(
        markers, "MOTION_WINDOW_START", "prefix")
    motion_end_marker = _marker_by_name(
        markers, "MOTION_WINDOW_END", "middle_suffix")
    flow_marker = _marker_by_name(
        markers, "FLOW_MIDDLE_SUFFIX_PASS", "middle_suffix")
    _require(support_marker["sequence"] < motion_start_marker["sequence"] <
             motion_end_marker["sequence"] < flow_marker["sequence"],
             "TIMING_MARKER_ORDER")
    main_motion = main_value["markers"]["motion_window"]
    _require(
        main_motion["printed_start_monotonic_ns"] ==
        int(motion_start_marker["line"].rsplit("=", 1)[1]) and
        main_motion["printed_end_monotonic_ns"] ==
        int(motion_end_marker["line"].rsplit("=", 1)[1]) and
        main_motion["observed_duration_ns"] ==
        motion_end_marker["monotonic_ns"] -
        motion_start_marker["monotonic_ns"],
        "MAIN_CERTIFICATE_MOTION_MARKER_BINDING")
    proc_value = _load_canonical_json(proc_evidence, "PROC_SNAPSHOT")
    process_intervals = _validate_process_snapshot(proc_value, run_id)
    prefix_text = _decode_log(prefix_stdout, "PREFIX_STDOUT")
    middle_text = _decode_log(middle_stdout, "MIDDLE_STDOUT")
    wall = _unique_motion_window(
        prefix_text, middle_text, process_intervals, support_marker,
        motion_start_marker, motion_end_marker, flow_marker)
    _require(metadata_evidence.path == rosbag_evidence.path.parent / "metadata.yaml",
             "ROSBAG_METADATA_LOCATION")
    metadata_summary = _validate_metadata(metadata_evidence, rosbag_evidence.path)
    sim = _analyze_rosbag(
        rosbag_evidence, metadata_summary, support_marker, flow_marker,
        len(markers))
    inputs = {
        "main_certificate": _binding(main_certificate),
        "markers": _binding(markers_evidence),
        "middle_stdout": _binding(middle_stdout),
        "prefix_stdout": _binding(prefix_stdout),
        "proc_snapshot": _binding(proc_evidence),
        "rosbag": _binding(rosbag_evidence),
        "rosbag_metadata": _binding(metadata_evidence),
    }
    return {
        "schema": SCHEMA,
        "verifier_version": VERIFIER_VERSION,
        "run_id": run_id,
        "inputs": inputs,
        "main_certificate": {
            "schema": main_value["schema"],
            "sha256": main_certificate.sha256,
            "valid": True,
            "status": "PASS",
        },
        "marker_binding": {
            "marker_count": len(markers),
            "marker_transcript_sha256": _canonical_hash(list(markers)),
            "support_preload": {
                "bag_timestamp_ns": support_marker["bag_timestamp_ns"],
                "monotonic_ns": support_marker["monotonic_ns"],
                "sequence": support_marker["sequence"],
            },
            "motion_window_start": {
                "bag_timestamp_ns": motion_start_marker["bag_timestamp_ns"],
                "monotonic_ns": motion_start_marker["monotonic_ns"],
                "sequence": motion_start_marker["sequence"],
            },
            "motion_window_end": {
                "bag_timestamp_ns": motion_end_marker["bag_timestamp_ns"],
                "monotonic_ns": motion_end_marker["monotonic_ns"],
                "sequence": motion_end_marker["sequence"],
            },
            "flow_middle_suffix_pass": {
                "bag_timestamp_ns": flow_marker["bag_timestamp_ns"],
                "monotonic_ns": flow_marker["monotonic_ns"],
                "sequence": flow_marker["sequence"],
            },
        },
        "process_binding": process_intervals,
        "wall_time": {
            **wall,
            "duration_seconds": wall["duration_ns"] / 1_000_000_000,
            "maximum_duration_ns": MAX_DURATION_NS,
            "unique_start_marker": True,
            "unique_end_marker": True,
            "passed": True,
        },
        "simulation_time": {
            **sim,
            "duration_upper_bound_seconds":
                sim["duration_upper_bound_ns"] / 1_000_000_000,
            "maximum_duration_ns": MAX_DURATION_NS,
            "mapping": (
                "last /clock stored at-or-before SUPPORT_PRELOAD; first "
                "/clock stored at-or-after FLOW_MIDDLE_SUFFIX_PASS"
            ),
            "conservative_upper_bound": True,
            "passed": True,
        },
        "slow_mode": False,
        "valid": True,
        "status": "PASS",
    }


def _validate_output_parent(path: Path) -> None:
    _require(path.is_absolute() and os.path.normpath(str(path)) == str(path),
             "OUTPUT_PATH")
    parent = path.parent
    try:
        metadata = parent.lstat()
        resolved = parent.resolve(strict=True)
    except OSError as error:
        raise Fast90CertificateError(f"OUTPUT_PARENT:{error}") from error
    _require(stat.S_ISDIR(metadata.st_mode) and not parent.is_symlink() and
             resolved == parent and metadata.st_uid == os.geteuid(),
             "OUTPUT_PARENT_IDENTITY")
    _require(not path.exists() and not path.is_symlink(), "OUTPUT_EXISTS")


def _atomic_publish_no_replace(path: Path, raw: bytes) -> None:
    _validate_output_parent(path)
    descriptor = -1
    temporary: Path | None = None
    linked = False
    try:
        descriptor, name = tempfile.mkstemp(
            prefix=f".{path.name}.stage.", dir=path.parent)
        temporary = Path(name)
        offset = 0
        while offset < len(raw):
            written = os.write(descriptor, raw[offset:])
            _require(written > 0, "OUTPUT_SHORT_WRITE")
            offset += written
        os.fsync(descriptor)
        os.fchmod(descriptor, 0o400)
        os.close(descriptor)
        descriptor = -1
        os.link(temporary, path, follow_symlinks=False)
        linked = True
        temporary.unlink()
        temporary = None
        directory_descriptor = os.open(
            path.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) |
            getattr(os, "O_CLOEXEC", 0))
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
        published = path.lstat()
        _require(stat.S_ISREG(published.st_mode) and
                 stat.S_IMODE(published.st_mode) == 0o400 and
                 published.st_nlink == 1 and
                 published.st_uid == os.geteuid(), "OUTPUT_IDENTITY")
    except Fast90CertificateError:
        if linked:
            try:
                path.unlink()
            except OSError:
                pass
        raise
    except OSError as error:
        if linked:
            try:
                path.unlink()
            except OSError:
                pass
        raise Fast90CertificateError(
            f"OUTPUT_ATOMIC_NO_REPLACE:{error}") from error
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        if temporary is not None:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass


def assemble(
    *,
    main_certificate_path: Path,
    main_certificate_sha256: str,
    prefix_stdout_path: Path,
    prefix_stdout_sha256: str,
    middle_stdout_path: Path,
    middle_stdout_sha256: str,
    markers_path: Path,
    markers_sha256: str,
    proc_snapshot_path: Path,
    proc_snapshot_sha256: str,
    rosbag_metadata_path: Path,
    rosbag_metadata_sha256: str,
    rosbag_path: Path,
    rosbag_sha256: str,
    output_path: Path,
) -> Tuple[Mapping[str, Any], str]:
    _validate_output_parent(output_path)
    main_certificate = _read_sealed_file(
        main_certificate_path, expected_sha256=main_certificate_sha256,
        maximum_bytes=MAX_JSON_BYTES, status="MAIN_CERTIFICATE_FILE")
    prefix_stdout = _read_sealed_file(
        prefix_stdout_path, expected_sha256=prefix_stdout_sha256,
        maximum_bytes=MAX_LOG_BYTES, status="PREFIX_STDOUT_FILE")
    middle_stdout = _read_sealed_file(
        middle_stdout_path, expected_sha256=middle_stdout_sha256,
        maximum_bytes=MAX_LOG_BYTES, status="MIDDLE_STDOUT_FILE")
    markers = _read_sealed_file(
        markers_path, expected_sha256=markers_sha256,
        maximum_bytes=MAX_JSON_BYTES, status="MARKERS_FILE")
    proc_snapshot = _read_sealed_file(
        proc_snapshot_path, expected_sha256=proc_snapshot_sha256,
        maximum_bytes=MAX_JSON_BYTES, status="PROC_SNAPSHOT_FILE")
    metadata = _read_sealed_file(
        rosbag_metadata_path, expected_sha256=rosbag_metadata_sha256,
        maximum_bytes=MAX_METADATA_BYTES, status="ROSBAG_METADATA_FILE")
    rosbag = _read_sealed_file(
        rosbag_path, expected_sha256=rosbag_sha256,
        maximum_bytes=MAX_DB_BYTES, status="ROSBAG_FILE")
    value = _build_value(
        main_certificate=main_certificate, prefix_stdout=prefix_stdout,
        middle_stdout=middle_stdout, markers_evidence=markers,
        proc_evidence=proc_snapshot, metadata_evidence=metadata,
        rosbag_evidence=rosbag)
    raw = _canonical_bytes(value)
    digest = hashlib.sha256(raw).hexdigest()
    _atomic_publish_no_replace(output_path, raw)
    return value, digest


def verify(
    *, certificate_path: Path, certificate_sha256: str,
) -> Mapping[str, Any]:
    certificate = _read_sealed_file(
        certificate_path, expected_sha256=certificate_sha256,
        maximum_bytes=MAX_JSON_BYTES, status="FAST90_CERTIFICATE_FILE")
    value = _load_canonical_json(certificate, "FAST90_CERTIFICATE")
    _require(value.get("schema") == SCHEMA and
             value.get("verifier_version") == VERIFIER_VERSION and
             value.get("valid") is True and value.get("status") == "PASS",
             "FAST90_CERTIFICATE_SHAPE")
    inputs = value.get("inputs")
    _require(type(inputs) is dict and set(inputs) == {
        "main_certificate", "prefix_stdout", "middle_stdout", "markers",
        "proc_snapshot", "rosbag_metadata", "rosbag",
    }, "FAST90_CERTIFICATE_INPUTS")
    evidence: Dict[str, FileEvidence] = {}
    limits = {
        "main_certificate": MAX_JSON_BYTES,
        "prefix_stdout": MAX_LOG_BYTES,
        "middle_stdout": MAX_LOG_BYTES,
        "markers": MAX_JSON_BYTES,
        "proc_snapshot": MAX_JSON_BYTES,
        "rosbag_metadata": MAX_METADATA_BYTES,
        "rosbag": MAX_DB_BYTES,
    }
    for identifier, binding_value in inputs.items():
        binding = _validate_binding(
            binding_value, f"VERIFY_INPUT_{identifier}")
        evidence[identifier] = _read_sealed_file(
            Path(binding["path"]), expected_sha256=binding["sha256"],
            maximum_bytes=limits[identifier],
            status=f"VERIFY_INPUT_{identifier}")
        _require(_binding(evidence[identifier]) == binding,
                 f"VERIFY_INPUT_{identifier}_BINDING")
    recomputed = _build_value(
        main_certificate=evidence["main_certificate"],
        prefix_stdout=evidence["prefix_stdout"],
        middle_stdout=evidence["middle_stdout"],
        markers_evidence=evidence["markers"],
        proc_evidence=evidence["proc_snapshot"],
        metadata_evidence=evidence["rosbag_metadata"],
        rosbag_evidence=evidence["rosbag"],
    )
    _require(value == recomputed, "FAST90_CERTIFICATE_RECOMPUTE_MISMATCH")
    _require(certificate.raw == _canonical_bytes(recomputed),
             "FAST90_CERTIFICATE_BYTES_MISMATCH")
    return value


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    assemble_parser = commands.add_parser("assemble")
    assemble_parser.add_argument("--main-certificate", type=Path, required=True)
    assemble_parser.add_argument("--main-certificate-sha256", required=True)
    assemble_parser.add_argument("--prefix-stdout", type=Path, required=True)
    assemble_parser.add_argument("--prefix-stdout-sha256", required=True)
    assemble_parser.add_argument("--middle-stdout", type=Path, required=True)
    assemble_parser.add_argument("--middle-stdout-sha256", required=True)
    assemble_parser.add_argument("--markers", type=Path, required=True)
    assemble_parser.add_argument("--markers-sha256", required=True)
    assemble_parser.add_argument("--proc-snapshot", type=Path, required=True)
    assemble_parser.add_argument("--proc-snapshot-sha256", required=True)
    assemble_parser.add_argument("--rosbag-metadata", type=Path, required=True)
    assemble_parser.add_argument("--rosbag-metadata-sha256", required=True)
    assemble_parser.add_argument("--rosbag", type=Path, required=True)
    assemble_parser.add_argument("--rosbag-sha256", required=True)
    assemble_parser.add_argument("--output", type=Path, required=True)
    verify_parser = commands.add_parser("verify")
    verify_parser.add_argument("--certificate", type=Path, required=True)
    verify_parser.add_argument("--certificate-sha256", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "assemble":
            value, digest = assemble(
                main_certificate_path=args.main_certificate,
                main_certificate_sha256=args.main_certificate_sha256,
                prefix_stdout_path=args.prefix_stdout,
                prefix_stdout_sha256=args.prefix_stdout_sha256,
                middle_stdout_path=args.middle_stdout,
                middle_stdout_sha256=args.middle_stdout_sha256,
                markers_path=args.markers,
                markers_sha256=args.markers_sha256,
                proc_snapshot_path=args.proc_snapshot,
                proc_snapshot_sha256=args.proc_snapshot_sha256,
                rosbag_metadata_path=args.rosbag_metadata,
                rosbag_metadata_sha256=args.rosbag_metadata_sha256,
                rosbag_path=args.rosbag,
                rosbag_sha256=args.rosbag_sha256,
                output_path=args.output,
            )
            print(
                "CANDIDATE3_FAST90_CERTIFICATE_831=PASS action=assemble "
                f"run_id={value['run_id']} sha256={digest} "
                f"wall_seconds={value['wall_time']['duration_seconds']:.9f} "
                "sim_upper_seconds="
                f"{value['simulation_time']['duration_upper_bound_seconds']:.9f} "
                "valid=true"
            )
        elif args.command == "verify":
            value = verify(
                certificate_path=args.certificate,
                certificate_sha256=args.certificate_sha256,
            )
            print(
                "CANDIDATE3_FAST90_CERTIFICATE_831=PASS action=verify "
                f"run_id={value['run_id']} "
                f"wall_seconds={value['wall_time']['duration_seconds']:.9f} "
                "sim_upper_seconds="
                f"{value['simulation_time']['duration_upper_bound_seconds']:.9f} "
                "valid=true"
            )
        else:  # pragma: no cover - argparse enforces command choices.
            raise Fast90CertificateError("UNKNOWN_COMMAND")
        return 0
    except Fast90CertificateError as error:
        print(f"CANDIDATE3_FAST90_CERTIFICATE_831=FAIL:{error}",
              file=os.sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
