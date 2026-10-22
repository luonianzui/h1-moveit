#!/usr/bin/python3
"""Fast-831 outer wrapper for the unchanged Gazebo candidate-3 flow.

``prefix`` first preloads the sealed support plugins, then executes the
certified q0->q90 path and exits while the right hand keeps grasping the
handle.  ``middle-and-suffix`` admits and runs the real eleven-phase middle,
requires the same fresh E0 seam before and after it, runs q90->q0, verifies
the complete released launch seam, and finally asks the support orchestrator
to certify the full stopped workcell.  The wrapper deliberately knows no
middle phase ranges or direct Gazebo support topics.
"""

from __future__ import annotations

import argparse
import hashlib
import hmac
import math
import os
import selectors
import socket
import stat
import subprocess
import sys
import time
import types
from pathlib import Path
from dataclasses import dataclass
from typing import Any, Sequence, Tuple


class FlowError(RuntimeError):
    """A fail-closed wrapper error."""


MIN_SUPPORT_TIMEOUT_S = 3.0
MAX_SUPPORT_TIMEOUT_S = 30.0
MIN_MIDDLE_SPEED_SCALE = 0.05
MAX_RIGHT_TIME_SCALE = 10.0
MAX_DISCOVERY_TIMEOUT_S = 60.0
MAX_WALL_TIMEOUT_FACTOR = 100.0
# The support CLI timeout bounds each child command; preload/finalize are
# finite multi-command operations and receive this separate outer bound.
SUPPORT_PROCESS_WALL_TIMEOUT_S = 300.0
MIDDLE_PROCESS_TIMEOUT_S = 7200.0
MAX_CAPTURE_BYTES = 1024 * 1024
FAST_SUPPORT_STARTUP_TIMEOUT_S = 60.0
FAST_SUPPORT_REQUEST_TIMEOUT_S = 60.0
FAST_SUPPORT_SHUTDOWN_TIMEOUT_S = 10.0
FAST_SUPPORT_RESPONSE_BYTES = 4096

FAST_SUPPORT_MARKERS = {
    "door-attach": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=door-attach "
        "door=attached cube=detached boundary=E0_ENTRY"),
    "cube-attach": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=cube-attach "
        "door=attached cube=attached boundary=CUBE_CONTACT"),
    "cube-detach": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=cube-detach "
        "door=attached cube=detached boundary=CUBE_RELEASE"),
    "door-detach": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=door-detach "
        "door=detached cube=detached boundary=E0_RETURN"),
    "finalize": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=finalize door=q0 "
        "handle=q0 latch=locked supports=detached robot=launch "
        "publishers=0"),
    "shutdown": (
        "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=shutdown "
        "execution=none"),
}


@dataclass
class FastSupportDaemon:
    process: subprocess.Popen[str] | None
    socket_path: Path
    ready_line: str


def _require(condition: bool, status: str) -> None:
    if not condition:
        raise FlowError(status)


def _identity(metadata: os.stat_result) -> Tuple[int, int, int, int, int]:
    return (metadata.st_dev, metadata.st_ino, metadata.st_size,
            metadata.st_mtime_ns, metadata.st_ctime_ns)


def _validate_sealed_file(
    path: Path, expected_sha256: str, mode: int, *, capture: bool,
) -> bytes | None:
    _require(path.is_absolute(), "SEALED_PATH_NOT_ABSOLUTE")
    _require(len(expected_sha256) == 64 and
             all(character in "0123456789abcdef" for character in expected_sha256),
             "SEALED_EXPECTED_SHA256")
    try:
        metadata = path.lstat()
    except OSError as error:
        raise FlowError(f"SEALED_LSTAT:{error}") from error
    _require(stat.S_ISREG(metadata.st_mode) and not path.is_symlink(),
             "SEALED_NOT_REGULAR")
    _require(metadata.st_uid == os.geteuid(), "SEALED_OWNER")
    _require(stat.S_IMODE(metadata.st_mode) == mode, "SEALED_MODE")
    _require(metadata.st_nlink == 1 and metadata.st_size > 0, "SEALED_IDENTITY")
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        raise FlowError(f"SEALED_OPEN:{error}") from error
    try:
        before = os.fstat(descriptor)
        _require(_identity(before) == _identity(metadata), "SEALED_RACE_BEFORE")
        digest = hashlib.sha256()
        chunks = [] if capture else None
        remaining = before.st_size
        while remaining:
            block = os.read(descriptor, min(1024 * 1024, remaining))
            _require(bool(block), "SEALED_SHORT_READ")
            digest.update(block)
            if chunks is not None:
                chunks.append(block)
            remaining -= len(block)
        _require(not os.read(descriptor, 1), "SEALED_GREW")
        _require(_identity(before) == _identity(os.fstat(descriptor)),
                 "SEALED_RACE_AFTER")
    finally:
        os.close(descriptor)
    _require(hmac.compare_digest(digest.hexdigest(), expected_sha256),
             "SEALED_SHA256")
    if chunks is None:
        return None
    return b"".join(chunks)


def _sealed_file(path: Path, expected_sha256: str, mode: int) -> None:
    _validate_sealed_file(path, expected_sha256, mode, capture=False)


def _read_sealed_source(
    path: Path, expected_sha256: str, mode: int = 0o500,
) -> bytes:
    source = _validate_sealed_file(
        path, expected_sha256, mode, capture=True)
    _require(type(source) is bytes, "SEALED_SOURCE_BYTES")
    return source


def _exec_verified_module(
    module_name: str, package_name: str, source: Path, source_bytes: bytes,
) -> Any:
    """Execute only the already verified bytes, never a path loader or pyc."""
    try:
        code = compile(source_bytes, str(source), "exec", dont_inherit=True)
    except (SyntaxError, ValueError, TypeError) as error:
        raise FlowError(f"VERIFIED_SOURCE_COMPILE:{module_name}:{error}") from error
    module = types.ModuleType(module_name)
    module.__file__ = str(source)
    module.__package__ = package_name
    module.__loader__ = None
    module.__spec__ = None
    previous = sys.modules.get(module_name)
    sys.modules[module_name] = module
    try:
        exec(code, module.__dict__)
    except BaseException as error:
        if previous is None:
            sys.modules.pop(module_name, None)
        else:
            sys.modules[module_name] = previous
        raise FlowError(f"VERIFIED_SOURCE_EXEC:{module_name}:{error}") from error
    return module


def _install_authority_package() -> Any:
    """Install an inert package shell without importing unverified init code."""
    package = types.ModuleType("h1_2_vision")
    package.__package__ = "h1_2_vision"
    package.__path__ = []
    package.__loader__ = None
    package.__spec__ = None
    sys.modules[package.__name__] = package
    return package


def _load_right_executor(args: argparse.Namespace) -> Any:
    source = args.right_executor
    source_bytes = _read_sealed_source(
        source, args.right_executor_sha256, 0o500)
    _require(source.resolve() == source, "RIGHT_EXECUTOR_PATH_ALIAS")
    return _exec_verified_module(
        "h1_right_handle_executor_828_bound", "", source, source_bytes)


def _right_arguments(args: argparse.Namespace, direction: str) -> list[str]:
    return [
        "--workspace", str(args.workspace),
        "--manifest", str(args.right_manifest),
        "--manifest-sha256", args.right_manifest_sha256,
        "--direction", direction,
        "--expected-direction", direction,
        "--time-scale", str(args.right_time_scale),
        "--discovery-timeout", str(args.discovery_timeout),
        "--wall-timeout-factor", str(args.wall_timeout_factor),
        "--execute",
    ]


def _run_right(args: argparse.Namespace, direction: str) -> None:
    module = _load_right_executor(args)
    status = module.main(_right_arguments(args, direction))
    _require(status == 0, f"RIGHT_{direction.upper()}_FAILED")


def _check_right_e0(args: argparse.Namespace, label: str) -> None:
    module = _load_right_executor(args)
    try:
        module.check_q90_seam(
            args.workspace, args.right_manifest,
            args.right_manifest_sha256, args.discovery_timeout)
    except Exception as error:
        raise FlowError(f"Q90_E0_{label}:{error}") from error


def _check_right_launch(args: argparse.Namespace) -> None:
    module = _load_right_executor(args)
    try:
        module.check_launch_seam(
            args.workspace, args.right_manifest,
            args.right_manifest_sha256, args.discovery_timeout)
    except Exception as error:
        raise FlowError(f"SUFFIX_TERMINAL_LAUNCH:{error}") from error


def _admit_middle(args: argparse.Namespace) -> None:
    # Bind both ROS-free source dependencies before importing them.  The
    # import paths must resolve to the exact sealed files named on the CLI;
    # this prevents an alternate PYTHONPATH copy from becoming the semantic
    # authority after a different file was checked.
    contract_bytes = _read_sealed_source(
        args.middle_contract, args.middle_contract_sha256, 0o500)
    admission_bytes = _read_sealed_source(
        args.middle_admission, args.middle_admission_sha256, 0o500)
    _require(args.middle_contract.resolve() == args.middle_contract,
             "MIDDLE_CONTRACT_PATH_ALIAS")
    _require(args.middle_admission.resolve() == args.middle_admission,
             "MIDDLE_ADMISSION_PATH_ALIAS")
    authority_package = _install_authority_package()
    contract = _exec_verified_module(
        "h1_2_vision.candidate3_middle_execution_contract_828",
        "h1_2_vision", args.middle_contract, contract_bytes)
    setattr(authority_package,
            "candidate3_middle_execution_contract_828", contract)
    admission = _exec_verified_module(
        "h1_2_vision.candidate3_middle_admission",
        "h1_2_vision", args.middle_admission, admission_bytes)
    setattr(authority_package, "candidate3_middle_admission", admission)
    _require(Path(contract.__file__).resolve() == args.middle_contract,
             "MIDDLE_CONTRACT_IMPORT_IDENTITY")
    _require(Path(admission.__file__).resolve() == args.middle_admission,
             "MIDDLE_ADMISSION_IMPORT_IDENTITY")
    try:
        admission.admit_result(args.result, args.result_sha256)
    except Exception as error:
        raise FlowError(f"MIDDLE_ADMISSION:{error}") from error


def _kill_process_group(process: subprocess.Popen) -> None:
    if process.poll() is None:
        try:
            os.killpg(process.pid, 9)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            pass


def _fast_support_socket_path() -> Path:
    directory = Path(f"/run/user/{os.geteuid()}")
    try:
        metadata = directory.lstat()
        resolved = directory.resolve(strict=True)
    except OSError as error:
        raise FlowError(f"FAST_SUPPORT_RUNTIME_DIR:{error}") from error
    _require(
        resolved == directory and stat.S_ISDIR(metadata.st_mode) and
        not directory.is_symlink() and metadata.st_uid == os.geteuid() and
        stat.S_IMODE(metadata.st_mode) == 0o700,
        "FAST_SUPPORT_RUNTIME_DIR_IDENTITY")
    path = directory / (
        f"h1fs831_{os.getpid()}_{time.monotonic_ns():x}.sock")
    _require(len(os.fsencode(path)) <= 100 and
             not path.exists() and not path.is_symlink(),
             "FAST_SUPPORT_SOCKET_PREEXISTING")
    return path


def _own_start_ticks() -> int:
    try:
        raw = Path("/proc/self/stat").read_text(encoding="utf-8")
        value = int(raw.rsplit(") ", 1)[1].split()[19])
    except (OSError, IndexError, ValueError) as error:
        raise FlowError(f"FAST_SUPPORT_OWNER_IDENTITY:{error}") from error
    _require(value > 0, "FAST_SUPPORT_OWNER_START_TICKS")
    return value


def _start_fast_support_daemon(args: argparse.Namespace) -> FastSupportDaemon:
    _sealed_file(
        args.fast_support_daemon, args.fast_support_daemon_sha256, 0o500)
    socket_path = _fast_support_socket_path()
    command = [
        str(args.fast_support_daemon), "serve",
        "--socket", str(socket_path),
        "--owner-pid", str(os.getpid()),
        "--owner-start-ticks", str(_own_start_ticks()),
        "--support-orchestrator", str(args.support_orchestrator),
        "--support-orchestrator-sha256", args.support_orchestrator_sha256,
        "--manifest", str(args.support_manifest),
        "--manifest-sha256", args.support_manifest_sha256,
        "--certificate", str(args.support_certificate),
        "--certificate-sha256", args.support_certificate_sha256,
        "--workspace", str(args.workspace),
        "--timeout", str(args.support_timeout),
        "--startup-timeout", str(FAST_SUPPORT_STARTUP_TIMEOUT_S),
        "--request-timeout", "5.0",
        "--telemetry-max-age", "1.0",
    ]
    try:
        process = subprocess.Popen(
            command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, bufsize=1,
            start_new_session=True)
    except OSError as error:
        raise FlowError(f"FAST_SUPPORT_DAEMON_SPAWN:{error}") from error
    selector = selectors.DefaultSelector()
    stdout_lines: list[str] = []
    stderr_lines: list[str] = []
    deadline = time.monotonic() + FAST_SUPPORT_STARTUP_TIMEOUT_S
    try:
        assert process.stdout is not None and process.stderr is not None
        selector.register(process.stdout, selectors.EVENT_READ, "stdout")
        selector.register(process.stderr, selectors.EVENT_READ, "stderr")
        while time.monotonic() < deadline:
            for key, _ in selector.select(timeout=0.1):
                line = key.fileobj.readline()
                if line == "":
                    continue
                encoded_size = sum(len(item.encode("utf-8")) for item in (
                    *stdout_lines, *stderr_lines, line))
                _require(encoded_size <= MAX_CAPTURE_BYTES,
                         "FAST_SUPPORT_DAEMON_STARTUP_OUTPUT_BUDGET")
                target = stdout_lines if key.data == "stdout" else stderr_lines
                target.append(line)
            _require(not stderr_lines,
                     "FAST_SUPPORT_DAEMON_STARTUP_STDERR:" +
                     "".join(stderr_lines).strip())
            if stdout_lines:
                expected = (
                    "H1_FAST_SUPPORT_DAEMON_831=READY "
                    f"door=detached cube=detached socket={socket_path}\n")
                _require(stdout_lines == [expected],
                         "FAST_SUPPORT_DAEMON_READY_MARKER")
                return FastSupportDaemon(
                    process=process, socket_path=socket_path,
                    ready_line=expected.rstrip("\n"))
            if process.poll() is not None:
                break
        raise FlowError(
            "FAST_SUPPORT_DAEMON_STARTUP:" +
            ("EXIT" if process.poll() is not None else "TIMEOUT"))
    except BaseException:
        _kill_process_group(process)
        try:
            if socket_path.exists() or socket_path.is_symlink():
                socket_path.unlink()
        except OSError:
            pass
        raise
    finally:
        selector.close()


def _fast_support_request(
    daemon: FastSupportDaemon, operation: str,
) -> str:
    _require(operation in FAST_SUPPORT_MARKERS,
             f"FAST_SUPPORT_OPERATION:{operation}")
    if daemon.process is not None:
        _require(daemon.process.poll() is None,
                 f"FAST_SUPPORT_DAEMON_NOT_RUNNING:{operation}")
    try:
        metadata = daemon.socket_path.lstat()
    except OSError as error:
        raise FlowError(
            f"FAST_SUPPORT_SOCKET_LSTAT:{operation}:{error}") from error
    _require(
        stat.S_ISSOCK(metadata.st_mode) and
        not daemon.socket_path.is_symlink() and
        metadata.st_uid == os.geteuid() and
        stat.S_IMODE(metadata.st_mode) == 0o600,
        f"FAST_SUPPORT_SOCKET_IDENTITY:{operation}")
    response = bytearray()
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(FAST_SUPPORT_REQUEST_TIMEOUT_S)
            client.connect(str(daemon.socket_path))
            client.sendall((operation + "\n").encode("ascii"))
            while b"\n" not in response:
                block = client.recv(
                    FAST_SUPPORT_RESPONSE_BYTES + 1 - len(response))
                _require(bool(block),
                         f"FAST_SUPPORT_RESPONSE_EOF:{operation}")
                response.extend(block)
                _require(len(response) <= FAST_SUPPORT_RESPONSE_BYTES,
                         f"FAST_SUPPORT_RESPONSE_BUDGET:{operation}")
    except OSError as error:
        raise FlowError(
            f"FAST_SUPPORT_REQUEST_IO:{operation}:{error}") from error
    _require(response.endswith(b"\n") and response.count(b"\n") == 1,
             f"FAST_SUPPORT_RESPONSE_FRAME:{operation}")
    try:
        marker = bytes(response[:-1]).decode("utf-8")
    except UnicodeDecodeError as error:
        raise FlowError(
            f"FAST_SUPPORT_RESPONSE_UTF8:{operation}:{error}") from error
    _require(marker == FAST_SUPPORT_MARKERS[operation],
             f"FAST_SUPPORT_RESPONSE_MARKER:{operation}:{marker}")
    return marker


def _finish_fast_support_daemon(daemon: FastSupportDaemon) -> None:
    _require(daemon.process is not None, "FAST_SUPPORT_DAEMON_NOT_OWNED")
    _fast_support_request(daemon, "shutdown")
    try:
        stdout, stderr = daemon.process.communicate(
            timeout=FAST_SUPPORT_SHUTDOWN_TIMEOUT_S)
    except subprocess.TimeoutExpired as error:
        _kill_process_group(daemon.process)
        raise FlowError(f"FAST_SUPPORT_DAEMON_SHUTDOWN:{error}") from error
    _require(daemon.process.returncode == 0,
             "FAST_SUPPORT_DAEMON_EXIT")
    _require(stderr == "", "FAST_SUPPORT_DAEMON_STDERR:" + stderr.strip())
    expected = [
        "H1_FAST_SUPPORT_DAEMON_831_REQUEST=PASS "
        f"command={operation}"
        for operation in (
            "door-attach", "cube-attach", "cube-detach", "door-detach",
            "finalize", "shutdown")
    ]
    _require(stdout.splitlines() == expected,
             "FAST_SUPPORT_DAEMON_TRANSCRIPT")
    _require(not daemon.socket_path.exists() and
             not daemon.socket_path.is_symlink(),
             "FAST_SUPPORT_SOCKET_NOT_REMOVED")


def _abort_fast_support_daemon(daemon: FastSupportDaemon) -> None:
    _require(daemon.process is not None, "FAST_SUPPORT_DAEMON_NOT_OWNED")
    _kill_process_group(daemon.process)
    try:
        if daemon.socket_path.exists() or daemon.socket_path.is_symlink():
            daemon.socket_path.unlink()
    except OSError:
        pass


def _wait_external_fast_support_socket(socket_path: Path) -> None:
    deadline = time.monotonic() + FAST_SUPPORT_STARTUP_TIMEOUT_S
    while time.monotonic() < deadline:
        try:
            metadata = socket_path.lstat()
        except FileNotFoundError:
            time.sleep(0.02)
            continue
        except OSError as error:
            raise FlowError(f"FAST_SUPPORT_SOCKET_READY:{error}") from error
        _require(
            stat.S_ISSOCK(metadata.st_mode) and
            not socket_path.is_symlink() and
            metadata.st_uid == os.geteuid() and
            stat.S_IMODE(metadata.st_mode) == 0o600,
            "FAST_SUPPORT_SOCKET_READY_IDENTITY")
        return
    raise FlowError("FAST_SUPPORT_SOCKET_READY_TIMEOUT")


def _finish_external_fast_support_daemon(daemon: FastSupportDaemon) -> None:
    _require(daemon.process is None, "FAST_SUPPORT_DAEMON_UNEXPECTED_OWNER")
    _fast_support_request(daemon, "shutdown")
    deadline = time.monotonic() + FAST_SUPPORT_SHUTDOWN_TIMEOUT_S
    while time.monotonic() < deadline:
        if (not daemon.socket_path.exists() and
                not daemon.socket_path.is_symlink()):
            return
        time.sleep(0.02)
    raise FlowError("FAST_SUPPORT_SOCKET_NOT_REMOVED")


def _run_bounded_process(
    command: Sequence[str], timeout_s: float, status: str,
) -> Tuple[int, str]:
    try:
        process = subprocess.Popen(
            list(command), stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True)
        try:
            stdout, stderr = process.communicate(timeout=timeout_s)
        except subprocess.TimeoutExpired as error:
            process.kill()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                pass
            raise FlowError(f"{status}_TIMEOUT:{error}") from error
    except OSError as error:
        raise FlowError(f"{status}_SPAWN:{error}") from error
    _require(len(stdout.encode("utf-8")) <= MAX_CAPTURE_BYTES,
             f"{status}_STDOUT_BUDGET")
    _require(len(stderr.encode("utf-8")) <= MAX_CAPTURE_BYTES,
             f"{status}_STDERR_BUDGET")
    if stderr:
        print(stderr, file=sys.stderr, end="")
    return process.returncode, stdout


def _run_middle(
    args: argparse.Namespace, daemon: FastSupportDaemon,
) -> None:
    _require(args.support_certificate.resolve() == args.certificate.resolve(),
             "SUPPORT_CERTIFICATE_PATH_IDENTITY")
    _require(args.support_certificate_sha256 == args.certificate_sha256,
             "SUPPORT_CERTIFICATE_SHA256_IDENTITY")
    _admit_middle(args)
    for path, digest in (
        (args.scene, args.scene_sha256),
        (args.plan, args.plan_sha256),
        (args.certificate, args.certificate_sha256),
        (args.result, args.result_sha256),
    ):
        _sealed_file(path, digest, 0o400)
    _sealed_file(args.middle_executor, args.middle_executor_sha256, 0o500)
    command = [
        str(args.middle_executor),
        "--workspace", str(args.workspace),
        "--scene", str(args.scene),
        "--scene-sha256", args.scene_sha256,
        "--plan", str(args.plan),
        "--plan-sha256", args.plan_sha256,
        "--certificate", str(args.certificate),
        "--certificate-sha256", args.certificate_sha256,
        "--result", str(args.result),
        "--result-sha256", args.result_sha256,
        "--middle-admission", str(args.middle_admission),
        "--middle-admission-sha256", args.middle_admission_sha256,
        "--middle-contract", str(args.middle_contract),
        "--middle-contract-sha256", args.middle_contract_sha256,
        "--support-orchestrator", str(args.support_orchestrator),
        "--support-orchestrator-sha256", args.support_orchestrator_sha256,
        "--support-manifest", str(args.support_manifest),
        "--support-manifest-sha256", args.support_manifest_sha256,
        "--support-certificate", str(args.support_certificate),
        "--support-certificate-sha256", args.support_certificate_sha256,
        "--speed-scale", str(args.middle_speed_scale),
        "--discovery-timeout", str(args.discovery_timeout),
        "--wall-timeout-factor", str(args.wall_timeout_factor),
        "--support-timeout", str(args.support_timeout),
        "--support-daemon-socket", str(daemon.socket_path),
        "--execute",
    ]
    returncode, stdout = _run_bounded_process(
        command, MIDDLE_PROCESS_TIMEOUT_S, "MIDDLE_EXECUTOR")
    _require(returncode == 0, "MIDDLE_EXECUTOR_FAILED")
    lines = [line for line in stdout.splitlines() if line]
    _require(bool(lines) and lines[-1] ==
             "CANDIDATE3_MIDDLE_828=PASS phases=11 "
             "right_gripper_commands=0 terminal_e0_held=true",
             "MIDDLE_EXECUTOR_MARKER")
    print(stdout, end="" if stdout.endswith("\n") else "\n")


def _run_support(args: argparse.Namespace, operation: str) -> None:
    _require(operation in ("preload", "finalize"), "SUPPORT_OPERATION")
    _sealed_file(
        args.support_orchestrator, args.support_orchestrator_sha256, 0o500)
    _sealed_file(args.support_manifest, args.support_manifest_sha256, 0o400)
    _sealed_file(
        args.support_certificate, args.support_certificate_sha256, 0o400)
    command = [
        str(args.support_orchestrator), operation,
        "--manifest", str(args.support_manifest),
        "--manifest-sha256", args.support_manifest_sha256,
        "--certificate", str(args.support_certificate),
        "--certificate-sha256", args.support_certificate_sha256,
        "--workspace", str(args.workspace),
        "--timeout", str(args.support_timeout),
    ]
    returncode, stdout = _run_bounded_process(
        command, SUPPORT_PROCESS_WALL_TIMEOUT_S,
        f"SUPPORT_{operation.upper()}")
    _require(returncode == 0, f"SUPPORT_{operation.upper()}_FAILED")
    expected = {
        "preload": "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=preload "
                   "door=detached cube=detached",
        "finalize": "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=finalize "
                    "door=q0 handle=q0 latch=locked supports=detached "
                    "robot=launch publishers=0",
    }[operation]
    _require(stdout.strip() == expected,
             f"SUPPORT_{operation.upper()}_MARKER")
    print(expected, flush=True)


def _add_right_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--right-executor", type=Path, required=True)
    parser.add_argument("--right-executor-sha256", required=True)
    parser.add_argument("--right-manifest", type=Path, required=True)
    parser.add_argument("--right-manifest-sha256", required=True)
    parser.add_argument("--right-time-scale", type=float, default=1.0)
    parser.add_argument("--discovery-timeout", type=float, default=10.0)
    parser.add_argument("--wall-timeout-factor", type=float, default=20.0)


def _add_support_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--support-orchestrator", type=Path, required=True)
    parser.add_argument("--support-orchestrator-sha256", required=True)
    parser.add_argument("--support-manifest", type=Path, required=True)
    parser.add_argument("--support-manifest-sha256", required=True)
    parser.add_argument("--support-certificate", type=Path, required=True)
    parser.add_argument("--support-certificate-sha256", required=True)
    parser.add_argument("--support-timeout", type=float, default=30.0)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="operation", required=True)
    prefix = commands.add_parser("prefix")
    _add_right_arguments(prefix)
    _add_support_arguments(prefix)
    prefix.add_argument(
        "--fast-support-daemon-socket", type=Path, required=True)
    finish = commands.add_parser("middle-and-suffix")
    _add_right_arguments(finish)
    _add_support_arguments(finish)
    finish.add_argument("--scene", type=Path, required=True)
    finish.add_argument("--scene-sha256", required=True)
    finish.add_argument("--plan", type=Path, required=True)
    finish.add_argument("--plan-sha256", required=True)
    finish.add_argument("--certificate", type=Path, required=True)
    finish.add_argument("--certificate-sha256", required=True)
    finish.add_argument("--result", type=Path, required=True)
    finish.add_argument("--result-sha256", required=True)
    finish.add_argument("--middle-executor", type=Path, required=True)
    finish.add_argument("--middle-executor-sha256", required=True)
    finish.add_argument("--middle-admission", type=Path, required=True)
    finish.add_argument("--middle-admission-sha256", required=True)
    finish.add_argument("--middle-contract", type=Path, required=True)
    finish.add_argument("--middle-contract-sha256", required=True)
    finish.add_argument("--middle-speed-scale", type=float, default=1.0)
    finish.add_argument("--fast-support-daemon", type=Path, required=True)
    finish.add_argument("--fast-support-daemon-sha256", required=True)
    finish.add_argument(
        "--fast-support-daemon-socket", type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        for name in ("workspace", "right_executor", "right_manifest"):
            _require(getattr(args, name).is_absolute(), f"{name.upper()}_NOT_ABSOLUTE")
        _require(math.isfinite(args.right_time_scale) and
                 abs(args.right_time_scale - 1.0) <= 1e-15,
                 "FAST_PROFILE_REQUIRES_RIGHT_SCALE_ONE")
        _require(math.isfinite(args.discovery_timeout) and
                 0.0 < args.discovery_timeout <= MAX_DISCOVERY_TIMEOUT_S,
                 "DISCOVERY_TIMEOUT")
        _require(math.isfinite(args.wall_timeout_factor) and
                 1.0 <= args.wall_timeout_factor <= MAX_WALL_TIMEOUT_FACTOR,
                 "WALL_TIMEOUT_FACTOR")
        _require(
            math.isfinite(args.support_timeout) and
            MIN_SUPPORT_TIMEOUT_S <= args.support_timeout <=
            MAX_SUPPORT_TIMEOUT_S,
            "SUPPORT_TIMEOUT")
        _require(args.fast_support_daemon_socket.is_absolute(),
                 "FAST_SUPPORT_DAEMON_SOCKET_NOT_ABSOLUTE")
        if args.operation == "prefix":
            print(
                "CANDIDATE3_FAST_831_FLOW_INVOCATION "
                f"monotonic_ns={time.monotonic_ns()}", flush=True)
            _run_support(args, "preload")
            _wait_external_fast_support_socket(
                args.fast_support_daemon_socket)
            print(
                "CANDIDATE3_FAST_831_MOTION_WINDOW=START "
                f"monotonic_ns={time.monotonic_ns()}", flush=True)
            _run_right(args, "prefix")
            print("CANDIDATE3_FLOW_828_PREFIX=PASS")
            return 0
        _require(math.isfinite(args.middle_speed_scale) and
                 abs(args.middle_speed_scale - 1.0) <= 1e-15,
                 "FAST_PROFILE_REQUIRES_MIDDLE_SCALE_ONE")
        _require(args.fast_support_daemon.is_absolute(),
                 "FAST_SUPPORT_DAEMON_NOT_ABSOLUTE")
        daemon = FastSupportDaemon(
            process=None, socket_path=args.fast_support_daemon_socket,
            ready_line="external-supervisor-owned")
        daemon_finished = False
        try:
            _check_right_e0(args, "BEFORE_MIDDLE")
            _run_middle(args, daemon)
            _check_right_e0(args, "AFTER_MIDDLE")
            _run_right(args, "suffix")
            _check_right_launch(args)
            print(_fast_support_request(daemon, "finalize"), flush=True)
            _finish_external_fast_support_daemon(daemon)
            daemon_finished = True
            print(
                "CANDIDATE3_FAST_831_MOTION_WINDOW=END "
                f"monotonic_ns={time.monotonic_ns()}", flush=True)
            print("CANDIDATE3_FLOW_828_MIDDLE_SUFFIX=PASS")
            return 0
        finally:
            if daemon.process is not None and not daemon_finished:
                _abort_fast_support_daemon(daemon)
    except FlowError as error:
        print(f"CANDIDATE3_FLOW_828=FAIL:{error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
