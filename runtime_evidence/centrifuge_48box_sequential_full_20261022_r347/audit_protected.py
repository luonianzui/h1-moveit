import hashlib
import json
from pathlib import Path
import sys


ROOT = Path("/home/liu/h1_moveit/h1_2_ros2_ws_claude_2026-07-30/h1_2_ros2_ws")
EVIDENCE = Path(__file__).resolve().parent
BASELINE = ROOT / "runtime_evidence/backup_920_release_20260920/archive_payload_sha256.json"


def digest(path):
    hasher = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            hasher.update(chunk)
    return hasher.hexdigest()


def main():
    phase = sys.argv[1]
    assert phase in ("before", "after")
    baseline = json.loads(BASELINE.read_text())
    prefixes = (
        "runtime_assets/", "runtime_evidence/", "src/h1_2_description/models/incubator/",
        "src/h1_2_navigation/", "src/h1_2_vision/", "src/h1_2_moveit_demo/",
        "src/h1_2_description/urdf/", "src/h1_2_description/meshes/",
    )
    protected = {name: expected for name, expected in baseline.items()
                 if name.startswith(prefixes)}
    plugin = "build/h1_2_moveit_demo/libh1_gazebo_compliant_grasp_system.so"
    protected[plugin] = baseline[plugin]
    failures = {}
    for name, expected in protected.items():
        path = ROOT / name
        actual = digest(path) if path.is_file() else "MISSING"
        if actual != expected:
            failures[name] = {"expected": expected, "actual": actual}
    backups = ROOT.parent.parent / "backups"
    for number in ("827", "828", "831", "901", "909", "914", "915", "920"):
        archive = backups / f"{number}.tar.zst"
        expected = (backups / f"{number}.tar.zst.sha256").read_text().split()[0]
        actual = digest(archive)
        if actual != expected or archive.stat().st_mode & 0o777 != 0o444:
            failures[str(archive)] = {"expected": expected, "actual": actual}
    archive_1008 = backups / "1008.tar.zst"
    sidecar_1008 = backups / "1008.tar.zst.sha256"
    expected_1008 = "92ae87377fe9a92922130df5ca30966f70551952a7243b87138976a78c0a2cf9"
    if (digest(archive_1008) != expected_1008 or
            sidecar_1008.read_text().split()[0] != expected_1008 or
            archive_1008.stat().st_mode & 0o777 != 0o444 or
            sidecar_1008.stat().st_mode & 0o777 != 0o444 or
            archive_1008.stat().st_nlink != 1 or sidecar_1008.stat().st_nlink != 1):
        failures[str(archive_1008)] = {"expected": expected_1008, "actual": digest(archive_1008)}
    archive_1009 = backups / "1009.tar.zst"
    sidecar_1009 = backups / "1009.tar.zst.sha256"
    expected_1009 = "9a746ba6b0f4a7fd871db63ef962cad0d58491207f0e102a85ce7f322c87ea26"
    if (digest(archive_1009) != expected_1009 or
            sidecar_1009.read_text().split()[0] != expected_1009 or
            archive_1009.stat().st_mode & 0o777 != 0o444 or
            sidecar_1009.stat().st_mode & 0o777 != 0o444 or
            archive_1009.stat().st_nlink != 1 or sidecar_1009.stat().st_nlink != 1):
        failures[str(archive_1009)] = {"expected": expected_1009, "actual": digest(archive_1009)}
    for identity in ("centrifuge_robot_press_20260922_r119",
                     "centrifuge_48box_right_lower_20261008_r154",
                     "centrifuge_48box_global_speed_20261009_r159",
                     "centrifuge_48box_full_cycle_20261009_r160",
                     "centrifuge_48box_full_cycle_20261009_r161",
                     "centrifuge_48box_full_cycle_20261009_r162",
                     "centrifuge_48box_full_cycle_20261009_r163",
                     "centrifuge_48box_full_cycle_20261009_r164",
                     "centrifuge_48box_full_cycle_20261009_r165",
                     "centrifuge_48box_full_cycle_20261009_r166",
                     "centrifuge_48box_full_cycle_20261009_r167",
                     "centrifuge_48box_rotor_physics_20261009_r168",
                     "centrifuge_48box_rotor_physics_20261009_r169",
                     "centrifuge_48box_rotor_physics_20261009_r170",
                     "centrifuge_48box_full_cycle_20261009_r171",
                     "centrifuge_48box_full_cycle_20261009_r172",
                     "centrifuge_48box_retrieval_actual_pose_20261009_r173",
                     "centrifuge_48box_retrieval_dense_20261009_r174",
                     "centrifuge_48box_full_cycle_20261009_r175",
                     "centrifuge_48box_full_cycle_20261009_r176",
                     "centrifuge_48box_full_cycle_20261009_r177",
                     "centrifuge_48box_full_cycle_20261009_r178",
                     "centrifuge_48box_stop_brake_20261009_r179",
                     "centrifuge_48box_stop_brake_20261009_r180",
                     "centrifuge_48box_full_cycle_20261009_r181",
                     "centrifuge_48box_retrieval_upright_20261009_r182",
                     "centrifuge_48box_full_cycle_20261009_r183",
                     "centrifuge_48box_stop_brake_20261009_r184",
                     "centrifuge_48box_full_cycle_20261009_r185",
                     "centrifuge_48box_stop_brake_20261010_r186",
                     "centrifuge_48box_extraction_local_z_20261010_r187",
                     "centrifuge_48box_full_cycle_20261010_r188",
                     "centrifuge_48box_position_brake_20261010_r189",
                     "centrifuge_48box_single_extraction_20261010_r190",
                     "centrifuge_48box_full_cycle_20261010_r191",
                     "centrifuge_48box_full_cycle_20261010_r192",
                     "centrifuge_48box_full_cycle_20261010_r193",
                     "centrifuge_48box_soft_grasp_20261010_r194",
                     "centrifuge_48box_full_cycle_20261010_r195",
                     "centrifuge_48box_clearance_unloading_20261010_r196",
                     "centrifuge_48box_terminal_unloading_20261010_r198",
                     "centrifuge_48box_leg_posture_unloading_20261010_r199",
                     "centrifuge_48box_setdown_braking_20261010_r201",
                     "centrifuge_48box_setdown_braking_20261010_r203",
                     "centrifuge_48box_setdown_braking_20261010_r205",
                     "centrifuge_48box_full_cycle_20261010_r206",
                     "centrifuge_48box_pilot_braking_20261010_r208",
                     "centrifuge_48box_fast_complete_20261010_r209",
                     "centrifuge_48box_bounded_speed_20261010_r210",
                     "centrifuge_48box_button_speed_20261010_r211",
                     "centrifuge_48box_lossless_complete_20261010_r212",
                     "centrifuge_48box_codec_complete_20261010_r213",
                     "centrifuge_48box_demo_complete_20261010_r214",
                     "build_cache_cleanup_20261010_r1",
                     "documentation_process_cleanup_20261010_r1",
                     "centrifuge_48box_speed_analysis_20261010_r215"):
        folder = ROOT / "runtime_evidence" / identity
        for line in (folder / "SHA256SUMS").read_text().splitlines():
            expected, relative = line.split(maxsplit=1)
            path = folder / relative.lstrip("*")
            if digest(path) != expected:
                failures[str(path)] = {"expected": expected, "actual": digest(path)}
    result = {"status": "PASS" if not failures else "FAIL",
              "protected_regular_files": len(protected),
              "backups_checked": 10, "failures": failures}
    with (EVIDENCE / f"protected_{phase}.json").open("x") as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result))
    assert not failures


if __name__ == "__main__":
    main()
