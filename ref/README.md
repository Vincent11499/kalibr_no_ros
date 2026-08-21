# ETHZ Kalibr reference snapshot

- Repository: `https://github.com/ethz-asl/kalibr`
- Commit: `1f60227442d25e36365ef5f72cd80b9666d73467`
- Local source used for the snapshot: `/home/czh/kalibr_workspace/src/kalibr`
- Snapshot directory: `ref/kalibr`
- License: BSD; the upstream `LICENSE` file is preserved in the snapshot.

Run `tools/verify_reference.py` to compare every file against
`ref/kalibr.sha256`. This directory is read-only project input and is used only
by the reference CMake presets. Editable production code belongs under
`src/kalibr`.
