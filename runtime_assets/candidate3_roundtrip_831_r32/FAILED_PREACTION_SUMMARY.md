# candidate3 roundtrip v22/r32 pre-action failure

- Date: 2026-09-15 CST
- Run id: `9537b53c6a9d9f7c006e6ecab5274531cc8f1a7d77bf64e440a2a3cd50ba5843`
- Result: `FAIL:PLUGIN_SHA256` before preparation and before flow.
- Controller commands issued: 0.
- No support attach/detach request was accepted and no manipulation motion ran.
- The short rosbag was stopped with SIGINT and has SQLite `integrity_check=ok`.
- Gazebo was stopped normally before the plugin file was restored.

The support contract requires
`eb85e8b833a6cbaa0fa26eab09e6a983ba2e36243bf37a12ebb9001c19e95d8b`,
but the main install symlink resolved to the D1a relinked file with digest
`1a81b048369ad16de5723d6ac987dfaedb8b00cfe4d10c90a0de2d5c41badcd7`.
The latter is preserved in this run root as
`libh1_gazebo_compliant_grasp_system.before_914_restore.so`.

r32 is sealed as a failed identity and must never be reused.
