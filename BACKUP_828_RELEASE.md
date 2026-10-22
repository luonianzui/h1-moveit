# 828 backup release contract

Release date: 2026-08-28 (Asia/Shanghai)

The non-overwrite archive targets for the accepted `final_run_09` work are:

```text
/home/liu/h1_moveit/backups/828.tar.zst
/home/liu/h1_moveit/backups/828.tar.zst.sha256
```

The archive contains the complete current `h1_2_ros2_ws/` workspace under that
single relative top-level directory. The adjacent sidecar is the authoritative
machine-verifiable archive hash; the archive does not embed its own digest.

Both files must be validated from same-directory temporary regular files and
published with no-replace hard links. The final archive and sidecar must be
regular files owned by `liu`, mode `0444`, link count 1. Existing archives,
including `826.3.tar.zst` and `827.tar.zst`, must remain unchanged.

The accepted runtime report is
`runtime_evidence/candidate3_828_full_flow_20260828/final_run_09/REPORT.md`.
Its video, postflight evidence, and deterministic `SHA256SUMS` are release
artifacts, not provisional diagnostics.
