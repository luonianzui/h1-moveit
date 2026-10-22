# Historical trace archives

The user authorized lossless compression of exactly 31 old
`run/loading_trace.jsonl` files from r161–r211 on 2026-10-14. The inventory and
per-file commit journal are in `runtime_evidence/trace_archive_20261014_r1/`.
Every archive is decoded completely by system gzip and checked against the
original SHA256SUMS, byte count and newline count before the original is removed.
The original checksum manifests, control sources, videos and reports are retained.

## Future simulation preparation and protection checks

From the workspace root, prefix the **new candidate's** preparation, test, or
verification command with this explicit launcher:

```bash
tools/trace_archive/run -- bash runtime_evidence/NEW_CANDIDATE/prepare_candidate.sh
tools/trace_archive/run -- /usr/bin/python3 runtime_evidence/NEW_CANDIDATE/verify_dual_protection.py before
tools/trace_archive/run -- /usr/bin/python3 runtime_evidence/NEW_CANDIDATE/verify_dual_protection.py after
```

`NEW_CANDIDATE` is a placeholder for a genuinely new run identity. Do not rerun
sealed historical identities or overwrite their outputs. The launcher does not
start any simulation itself. It changes the environment only for the supplied
command and its descendants; no shell startup files or system Python are changed.
Keep the inherited `H1_TRACE_ARCHIVE_REGISTRY` and `PYTHONPATH` when starting Python
children. Python `-I`/`-S`, clearing the environment, or shadowing `sitecustomize`
bypasses this opt-in support and is not a supported invocation.

For the exact 31 registered paths only, Python `open`, `io.open`, `Path.open`,
`read_text`, `read_bytes`, iteration and seek read the decompressed original bytes.
`stat`/`exists`/`is_file` expose the original logical file metadata. Original file
permissions are preserved in this logical view, not invented. The first access
validates the compressed SHA; consuming to EOF verifies original SHA, size and
newlines. Seeking performs complete logical verification first. Writes to the
registered historical raw paths are refused. All other paths use ordinary IO.

Archives reside in a separate `data/` directory, so the existing `trace_io.py`
does not see both a raw name and an adjacent `.gz` name. The registry is loaded
once per process; start a new process after a migration or registry update.

## Explicit integrity checks

```bash
/usr/bin/python3 tools/trace_archive/archive_cli.py doctor
/usr/bin/python3 tools/trace_archive/archive_cli.py verify-manifest \
  runtime_evidence/centrifuge_48box_full_cycle_20261009_r185/SHA256SUMS
```

The manifest checker validates **every** member against the original checksum,
including the decompressed trace. It does not skip missing members or change old
hashes. `--output NEW_FILE` saves a new report without overwriting an existing one.

This is a scoped Python IO adapter, not a filesystem mount. External `cat`, native
`sha256sum -c`, directory enumeration, `os.open`, mmap and raw file descriptors are
not virtualized. Use `verify-manifest` for old manifests containing migrated raw
paths, and `gzip -dc` on the registered archive for external streaming tools.
An ordinary `sha256sum -c` against those old manifests will report the raw path
missing; it does not mean the archived data failed its logical SHA check.

If an external tool requires a real raw file, first check free space, decompress
the chosen archive into a new temporary file, and verify its SHA and size against
`inventory.json` before use. Never remove the archive merely to restore a raw
copy. Native Gazebo plugins, scene resources and robot control data were not
changed by this migration.

## Migration audit and recovery

`inventory.json` records the fixed authorization scope and original manifests;
`registry.json` records the verified archive mapping. `journal.jsonl` is fsynced
after each commit and deletion. `migration_result.json` reports physical savings.
Compressed files use standard concatenated gzip members and can be decoded by
system gzip or Python gzip without this adapter.

`migrate.py` must run **without** the compatibility environment, with ordinary
non-optimized Python. It takes an exclusive lock, requires passing baseline
protection, processes files in increasing size, keeps a 128 MiB reserve, checks
source identity and readable process FD references, and never overwrites archives.
On resume it revalidates committed archives; an unfinished output is discarded
only after independently rehashing the still-present original. The scan of `/proc`
has recorded permission blind spots and is not a claim of universal visibility.

The original r273 protection chain is executed unchanged by
`verify_current_protection.py`, with its new output directory bound to this
migration's audit directory. Historical audits that write with `open('x')` still
require a fresh output identity; this adapter does not override their output rules.
