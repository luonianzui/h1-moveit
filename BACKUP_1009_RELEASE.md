# Backup 1009 Release Contract

Release date: 2026-10-09 (Asia/Shanghai). The user selected the unused number
1009 before the next requested live demonstration. The r159 Gazebo scene has
been paused after PASS and closed through normal SIGINT; all 114 r159 checksums
still match. No replay has started while this snapshot is being prepared.

Canonical targets:

```text
/home/liu/h1_moveit/backups/1009.tar.zst
/home/liu/h1_moveit/backups/1009.tar.zst.sha256
```

Archive the complete current workspace with the single safe relative root
`h1_2_ros2_ws/`. Preserve symbolic links and include all existing workspace
source, models, build/install trees, authorities, successful and failed run
evidence, reports, videos, rosbags, handoffs, and this pre-release contract.
The external backups directory is excluded. This snapshot includes r155-r159
and the r159 complete two-box loading PASS of 172.500 simulated seconds measured
from first executor state sample to PASS. Its absolute PASS clock is 185.491 s.

Both targets must be absent. Use unique temporary regular files in the backups
directory, validate them, and publish with no-replace hard links. Final archive
and sidecar must be ordinary liu:liu files, mode 0444, with one hard link each.

Check all nine protected backups (827, 828, 831, 901, 909, 914, 915, 920, 1008)
before and after publication. Inventory the frozen source without following
symlinks. Audit every archive member for safe unique relative paths, expected
root and permitted types; verify matching symlink targets and no descendants
under symlinks. Hash every regular archived payload against its source and
confirm the source inventory remains unchanged throughout creation and audit.
Run zstd integrity and sidecar checks, verify the archived r159 checksum list,
and fully decode the archived r154 recording.

The published digest and validation results belong in the sidecar and external
release report. Append actual publication results to live handoff documents
only after successful validation, avoiding a self-referential archive digest.
