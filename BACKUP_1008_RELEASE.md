# Backup 1008 Release Contract

Release date: 2026-10-08 (Asia/Shanghai). The user explicitly selected the
previously unused archive number `1008` after the fresh r154 Gazebo run placed
both 85%-scale 48box objects into their respective centrifuge baskets.

Canonical targets:

```text
/home/liu/h1_moveit/backups/1008.tar.zst
/home/liu/h1_moveit/backups/1008.tar.zst.sha256
```

The archive is a complete snapshot of the current workspace under the single
safe relative root `h1_2_ros2_ws/`. Preserve symbolic links as links. The
external `/home/liu/h1_moveit/backups/` directory is not part of the workspace
and must not be archived. This snapshot includes the r154 report, full event
trace, validated video, audit outputs, and this pre-release contract.

Both final targets must be absent before work begins. Create unique temporary
regular files in the backups directory; validate the archive and its sidecar
before publishing with no-replace hard links. Never overwrite an existing
archive, sidecar, prior backup, or temporary identity. Final archive and
sidecar must be regular files owned by `liu:liu`, mode `0444`, and link count 1.

Before release, inventory the source without following symlinks and check the
existing protected chain `827`, `828`, `831`, `901`, `909`, `914`, `915`, and
`920`. Verify every archived member for a safe unique relative path, the one
expected root, permitted member type, matching symlink target, and absence of
members beneath symlinks. Hash every archived regular-file payload and compare
it to the source; confirm the source inventory remains unchanged. Run `zstd -t`,
verify the sidecar with `sha256sum -c`, and fully decode the archived r154 MP4.
Repeat old-backup chain and final metadata checks after publication.

The actual archive digest, final size/counts, and post-publication results must
be written only to the adjacent sidecar and an external release report after
successful validation; do not put a self-referential digest in this contract.
This backup is a Gazebo workspace snapshot only and conveys no real-robot
authorization.
