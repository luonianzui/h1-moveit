"""Source and ordering tests for the two-gate 828 outer wrapper."""

import hashlib
import importlib.util
import os
import py_compile
import stat
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


WORKSPACE = Path(__file__).resolve().parents[3]
SOURCE = WORKSPACE / "src/h1_2_moveit_demo/scripts/h1_candidate3_flow_828.py"
ADMISSION_SOURCE = (
    WORKSPACE / "src/h1_2_vision/h1_2_vision/candidate3_middle_admission.py")
CONTRACT_SOURCE = (
    WORKSPACE /
    "src/h1_2_vision/h1_2_vision/candidate3_middle_execution_contract_828.py")
SPEC = importlib.util.spec_from_file_location("h1_candidate3_flow_828_test", SOURCE)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _right_cli(operation):
    return [
        operation,
        "--workspace", str(WORKSPACE),
        "--right-executor", "/tmp/right-executor",
        "--right-executor-sha256", "9" * 64,
        "--right-manifest", str(WORKSPACE / "manifest.json"),
        "--right-manifest-sha256", "0" * 64,
        "--support-orchestrator", "/tmp/support",
        "--support-orchestrator-sha256", "6" * 64,
        "--support-manifest", "/tmp/support-manifest",
        "--support-manifest-sha256", "7" * 64,
        "--support-certificate", "/tmp/support-certificate",
        "--support-certificate-sha256", "8" * 64,
    ]


def test_prefix_dispatches_only_certified_right_prefix(monkeypatch, capsys):
    calls = []
    monkeypatch.setattr(
        MODULE, "_run_support",
        lambda _args, operation: calls.append(operation))
    monkeypatch.setattr(MODULE, "_run_right",
                        lambda _args, direction: calls.append(direction))
    assert MODULE.main(_right_cli("prefix")) == 0
    assert calls == ["preload", "prefix"]
    assert "CANDIDATE3_FLOW_828_PREFIX=PASS" in capsys.readouterr().out


def test_middle_must_succeed_before_suffix(monkeypatch, capsys):
    calls = []
    monkeypatch.setattr(MODULE, "_check_right_e0",
                        lambda _args, label: calls.append(label))
    monkeypatch.setattr(MODULE, "_run_middle", lambda _args: calls.append("middle"))
    monkeypatch.setattr(MODULE, "_run_right",
                        lambda _args, direction: calls.append(direction))
    monkeypatch.setattr(MODULE, "_check_right_launch",
                        lambda _args: calls.append("launch"))
    monkeypatch.setattr(
        MODULE, "_run_support",
        lambda _args, operation: calls.append(operation))
    arguments = _right_cli("middle-and-suffix") + [
        "--scene", "/tmp/scene.cdr", "--scene-sha256", "1" * 64,
        "--plan", "/tmp/plan.json", "--plan-sha256", "2" * 64,
        "--certificate", "/tmp/cert.json", "--certificate-sha256", "3" * 64,
        "--result", "/tmp/result.json", "--result-sha256", "4" * 64,
        "--middle-executor", "/tmp/middle", "--middle-executor-sha256", "5" * 64,
        "--middle-admission", str(ADMISSION_SOURCE),
        "--middle-admission-sha256", "a" * 64,
        "--middle-contract", str(CONTRACT_SOURCE),
        "--middle-contract-sha256", "b" * 64,
    ]
    assert MODULE.main(arguments) == 0
    assert calls == [
        "BEFORE_MIDDLE", "middle", "AFTER_MIDDLE", "suffix", "launch",
        "finalize",
    ]
    assert "MIDDLE_SUFFIX=PASS" in capsys.readouterr().out


def test_middle_failure_prevents_suffix(monkeypatch):
    calls = []
    monkeypatch.setattr(MODULE, "_check_right_e0",
                        lambda _args, label: calls.append(label))

    def fail(_args):
        raise MODULE.FlowError("MIDDLE_FAILED")

    monkeypatch.setattr(MODULE, "_run_middle", fail)
    monkeypatch.setattr(MODULE, "_run_right",
                        lambda _args, direction: calls.append(direction))
    arguments = _right_cli("middle-and-suffix") + [
        "--scene", "/tmp/scene", "--scene-sha256", "1" * 64,
        "--plan", "/tmp/plan", "--plan-sha256", "2" * 64,
        "--certificate", "/tmp/cert", "--certificate-sha256", "3" * 64,
        "--result", "/tmp/result", "--result-sha256", "4" * 64,
        "--middle-executor", "/tmp/executor", "--middle-executor-sha256", "5" * 64,
        "--middle-admission", str(ADMISSION_SOURCE),
        "--middle-admission-sha256", "a" * 64,
        "--middle-contract", str(CONTRACT_SOURCE),
        "--middle-contract-sha256", "b" * 64,
    ]
    assert MODULE.main(arguments) == 1
    assert calls == ["BEFORE_MIDDLE"]


@pytest.mark.parametrize("timeout", [2.99, 30.01])
def test_support_timeout_outside_orchestrator_contract_is_rejected(
        monkeypatch, timeout):
    calls = []
    monkeypatch.setattr(
        MODULE, "_run_support",
        lambda *_args: calls.append("unexpected"))
    arguments = _right_cli("prefix") + [
        "--support-timeout", str(timeout),
    ]
    assert MODULE.main(arguments) == 1
    assert calls == []


@pytest.mark.parametrize(
    ("operation", "marker"),
    (
        (
            "preload",
            "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=preload "
            "door=detached cube=detached",
        ),
        (
            "finalize",
            "H1_SUPPORT_ORCHESTRATOR_828=PASS operation=finalize "
            "door=q0 handle=q0 latch=locked supports=detached "
            "robot=launch publishers=0",
        ),
    ),
)
def test_support_process_budget_is_separate_from_per_command_timeout(
        monkeypatch, operation, marker):
    sealed = []
    calls = []
    monkeypatch.setattr(
        MODULE, "_sealed_file",
        lambda path, digest, mode: sealed.append((path, digest, mode)))

    def run(command, timeout, status):
        calls.append((command, timeout, status))
        return 0, marker + "\n"

    monkeypatch.setattr(MODULE, "_run_bounded_process", run)
    args = SimpleNamespace(
        workspace=Path("/workspace"),
        support_orchestrator=Path("/support"),
        support_orchestrator_sha256="6" * 64,
        support_manifest=Path("/support-manifest"),
        support_manifest_sha256="7" * 64,
        support_certificate=Path("/support-certificate"),
        support_certificate_sha256="8" * 64,
        support_timeout=17.25,
    )
    MODULE._run_support(args, operation)
    assert sealed == [
        (Path("/support"), "6" * 64, 0o500),
        (Path("/support-manifest"), "7" * 64, 0o400),
        (Path("/support-certificate"), "8" * 64, 0o400),
    ]
    assert calls == [(
        [
            "/support", operation,
            "--manifest", "/support-manifest",
            "--manifest-sha256", "7" * 64,
            "--certificate", "/support-certificate",
            "--certificate-sha256", "8" * 64,
            "--workspace", "/workspace",
            "--timeout", "17.25",
        ],
        300.0,
        f"SUPPORT_{operation.upper()}",
    )]
    assert MODULE.SUPPORT_PROCESS_WALL_TIMEOUT_S == 300.0
    assert (MODULE.SUPPORT_PROCESS_WALL_TIMEOUT_S >
            MODULE.MAX_SUPPORT_TIMEOUT_S)


def test_middle_interface_is_admission_then_exact_four_file_cli(monkeypatch):
    events = []
    monkeypatch.setattr(MODULE, "_admit_middle",
                        lambda _args: events.append("admit"))
    monkeypatch.setattr(MODULE, "_sealed_file",
                        lambda path, digest, mode: events.append((path, digest, mode)))

    def run(command, timeout, status):
        events.append(("run", command, timeout, status))
        return (0, "CANDIDATE3_MIDDLE_828=PASS phases=11 "
                "right_gripper_commands=0 terminal_e0_held=true\n")

    monkeypatch.setattr(MODULE, "_run_bounded_process", run)
    args = SimpleNamespace(
        workspace=Path("/workspace"),
        scene=Path("/scene"), scene_sha256="1" * 64,
        plan=Path("/plan"), plan_sha256="2" * 64,
        certificate=Path("/certificate"), certificate_sha256="3" * 64,
        result=Path("/result"), result_sha256="4" * 64,
        middle_executor=Path("/executor"), middle_executor_sha256="5" * 64,
        middle_admission=Path("/admission"),
        middle_admission_sha256="a" * 64,
        middle_contract=Path("/contract"),
        middle_contract_sha256="b" * 64,
        support_orchestrator=Path("/support"),
        support_orchestrator_sha256="6" * 64,
        support_manifest=Path("/support-manifest"),
        support_manifest_sha256="7" * 64,
        support_certificate=Path("/certificate"),
        support_certificate_sha256="3" * 64,
        middle_speed_scale=1.0,
        discovery_timeout=10.0,
        wall_timeout_factor=20.0,
        support_timeout=30.0,
    )
    MODULE._run_middle(args)
    assert events[0] == "admit"
    command = events[-1][1]
    assert command == [
        "/executor", "--workspace", "/workspace",
        "--scene", "/scene", "--scene-sha256", "1" * 64,
        "--plan", "/plan", "--plan-sha256", "2" * 64,
        "--certificate", "/certificate",
        "--certificate-sha256", "3" * 64,
        "--result", "/result", "--result-sha256", "4" * 64,
        "--middle-admission", "/admission",
        "--middle-admission-sha256", "a" * 64,
        "--middle-contract", "/contract",
        "--middle-contract-sha256", "b" * 64,
        "--support-orchestrator", "/support",
        "--support-orchestrator-sha256", "6" * 64,
        "--support-manifest", "/support-manifest",
        "--support-manifest-sha256", "7" * 64,
        "--support-certificate", "/certificate",
        "--support-certificate-sha256", "3" * 64,
        "--speed-scale", "1.0", "--discovery-timeout", "10.0",
        "--wall-timeout-factor", "20.0", "--support-timeout", "30.0",
        "--execute",
    ]


def test_bounded_process_timeout_kills_entire_group(monkeypatch):
    events = []

    class Process:
        pid = 5151
        returncode = None

        def communicate(self, timeout):
            raise MODULE.subprocess.TimeoutExpired("child", timeout)

        def poll(self):
            return self.returncode

        def wait(self, timeout):
            self.returncode = -9
            return self.returncode

    def popen(*_args, **kwargs):
        assert kwargs["start_new_session"] is True
        return Process()

    monkeypatch.setattr(MODULE.subprocess, "Popen", popen)
    monkeypatch.setattr(
        MODULE.os, "killpg", lambda pid, signal: events.append((pid, signal)))
    with pytest.raises(MODULE.FlowError, match="CHILD_TIMEOUT"):
        MODULE._run_bounded_process(["/child"], 1.0, "CHILD")
    assert events == [(5151, 9)]


def test_middle_rejects_distinct_support_certificate_before_admission():
    args = SimpleNamespace(
        support_certificate=Path("/support-certificate"),
        certificate=Path("/certificate"),
        support_certificate_sha256="1" * 64,
        certificate_sha256="1" * 64,
    )
    with pytest.raises(MODULE.FlowError,
                       match="SUPPORT_CERTIFICATE_PATH_IDENTITY"):
        MODULE._run_middle(args)


def test_admission_binds_both_sealed_source_dependencies(monkeypatch):
    events = []
    package_name = "h1_2_vision"
    contract_name = (
        "h1_2_vision.candidate3_middle_execution_contract_828")
    admission_name = "h1_2_vision.candidate3_middle_admission"
    previous_modules = {
        name: sys.modules.get(name)
        for name in (package_name, contract_name, admission_name)
    }
    contract_bytes = (
        b"def validate_result(value):\n"
        b"    return {'verified': value}\n")
    admission_bytes = (
        b"from .candidate3_middle_execution_contract_828 import validate_result\n"
        b"def admit_result(path, digest):\n"
        b"    return validate_result((str(path), digest))\n")

    def read_source(path, digest, mode):
        events.append((path, digest, mode))
        if path == CONTRACT_SOURCE:
            return contract_bytes
        assert path == ADMISSION_SOURCE
        return admission_bytes

    monkeypatch.setattr(MODULE, "_read_sealed_source", read_source)
    args = SimpleNamespace(
        middle_contract=CONTRACT_SOURCE,
        middle_contract_sha256="b" * 64,
        middle_admission=ADMISSION_SOURCE,
        middle_admission_sha256="a" * 64,
        result=Path("/result"),
        result_sha256="4" * 64,
    )
    try:
        MODULE._admit_middle(args)
        package = sys.modules[package_name]
        contract = sys.modules[contract_name]
        admission = sys.modules[admission_name]
        assert package.__path__ == []
        assert package.__loader__ is None
        assert admission.validate_result is contract.validate_result
        assert contract.__loader__ is None
        assert admission.__loader__ is None
        assert events == [
            (CONTRACT_SOURCE, "b" * 64, 0o500),
            (ADMISSION_SOURCE, "a" * 64, 0o500),
        ]
    finally:
        for name, previous in previous_modules.items():
            if previous is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = previous


def _sealed_python_source(path, source):
    path.write_bytes(source)
    path.chmod(0o500)
    return hashlib.sha256(source).hexdigest()


def test_right_executor_ignores_matching_timestamp_pyc(tmp_path):
    source = tmp_path / "right_executor.py"
    malicious = b"VALUE = 'EVIL'\n"
    verified = b"VALUE = 'GOOD'\n"
    assert len(malicious) == len(verified)
    source.write_bytes(malicious)
    timestamp = source.stat()
    pyc = Path(importlib.util.cache_from_source(str(source)))
    py_compile.compile(str(source), cfile=str(pyc), doraise=True)
    source.write_bytes(verified)
    os.utime(source, ns=(timestamp.st_atime_ns, timestamp.st_mtime_ns))
    source.chmod(0o500)
    assert pyc.is_file()
    args = SimpleNamespace(
        right_executor=source,
        right_executor_sha256=hashlib.sha256(verified).hexdigest(),
    )
    module = MODULE._load_right_executor(args)
    assert module.VALUE == "GOOD"


def test_right_executor_does_not_use_importlib_path_loader(
        tmp_path, monkeypatch):
    source = tmp_path / "right_executor.py"
    digest = _sealed_python_source(source, b"VALUE = 'VERIFIED'\n")

    def forbidden_loader(*_args, **_kwargs):
        raise AssertionError("path loader must not run")

    monkeypatch.setattr(
        importlib.util, "spec_from_file_location", forbidden_loader)
    module = MODULE._load_right_executor(SimpleNamespace(
        right_executor=source, right_executor_sha256=digest))
    assert module.VALUE == "VERIFIED"
    assert module.__loader__ is None


def test_right_executor_executes_captured_bytes_after_path_changes(
        tmp_path, monkeypatch):
    source = tmp_path / "right_executor.py"
    verified = b"VALUE = 'VERIFIED'\n"
    replacement = b"VALUE = 'REPLACED'\n"
    digest = _sealed_python_source(source, verified)
    read_source = MODULE._read_sealed_source

    def read_then_replace(path, expected_sha256, mode):
        captured = read_source(path, expected_sha256, mode)
        path.chmod(0o700)
        path.write_bytes(replacement)
        path.chmod(0o500)
        return captured

    monkeypatch.setattr(MODULE, "_read_sealed_source", read_then_replace)
    module = MODULE._load_right_executor(SimpleNamespace(
        right_executor=source, right_executor_sha256=digest))
    assert source.read_bytes() == replacement
    assert module.VALUE == "VERIFIED"


def test_middle_authority_does_not_load_malicious_init_pyc(
        tmp_path, monkeypatch):
    package_directory = tmp_path / "h1_2_vision"
    package_directory.mkdir()
    init_source = package_directory / "__init__.py"
    malicious = b"raise RuntimeError('INIT_PYC_EXECUTED')\n"
    verified = b"PACKAGE_VALUE = 'GOOD'\n"
    width = max(len(malicious), len(verified))
    malicious = malicious.rstrip(b"\n").ljust(width - 1) + b"\n"
    verified = verified.rstrip(b"\n").ljust(width - 1) + b"\n"
    init_source.write_bytes(malicious)
    timestamp = init_source.stat()
    pyc = Path(importlib.util.cache_from_source(str(init_source)))
    py_compile.compile(str(init_source), cfile=str(pyc), doraise=True)
    init_source.write_bytes(verified)
    os.utime(
        init_source, ns=(timestamp.st_atime_ns, timestamp.st_mtime_ns))
    monkeypatch.syspath_prepend(str(tmp_path))

    names = (
        "h1_2_vision",
        "h1_2_vision.candidate3_middle_execution_contract_828",
        "h1_2_vision.candidate3_middle_admission",
    )
    previous = {name: sys.modules.pop(name, None) for name in names}
    contract_bytes = b"def validate_result(value):\n    return value\n"
    admission_bytes = (
        b"from .candidate3_middle_execution_contract_828 import validate_result\n"
        b"def admit_result(path, digest):\n"
        b"    return validate_result((path, digest))\n")

    def read_source(path, _digest, _mode):
        if path.name == "contract.py":
            return contract_bytes
        return admission_bytes

    monkeypatch.setattr(MODULE, "_read_sealed_source", read_source)
    args = SimpleNamespace(
        middle_contract=(tmp_path / "contract.py").resolve(),
        middle_contract_sha256="b" * 64,
        middle_admission=(tmp_path / "admission.py").resolve(),
        middle_admission_sha256="a" * 64,
        result=Path("/result"), result_sha256="4" * 64,
    )
    try:
        MODULE._admit_middle(args)
        package = sys.modules["h1_2_vision"]
        assert package.__loader__ is None
        assert package.__path__ == []
        assert not hasattr(package, "PACKAGE_VALUE")
    finally:
        for name in names:
            sys.modules.pop(name, None)
        for name, module in previous.items():
            if module is not None:
                sys.modules[name] = module


def test_semantic_authority_sources_are_owner_read_execute_only():
    for source in (ADMISSION_SOURCE, CONTRACT_SOURCE):
        metadata = source.lstat()
        assert stat.S_IMODE(metadata.st_mode) == 0o500
        assert metadata.st_nlink == 1


def test_source_does_not_guess_middle_phase_semantics_or_import_ros():
    source = SOURCE.read_text(encoding="utf-8")
    assert "import rclpy" not in source
    assert "trajectory_msgs" not in source
    for forbidden in (
        "door_pre_support", "cube_contact", "cube_carry_place",
        "door_close_supported", "right_release",
    ):
        assert forbidden not in source
    assert source.index("_admit_middle(args)") < source.index(
        "_run_bounded_process(\n        command, MIDDLE_PROCESS_TIMEOUT_S")
    assert "Path(__file__).with_name" not in source
    assert "start_new_session=True" in source
    assert "os.killpg(process.pid, 9)" in source
