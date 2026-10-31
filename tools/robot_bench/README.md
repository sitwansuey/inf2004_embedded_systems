# Robot bench tools

These helpers are optional diagnostics. They resolve the repository relative
to their own location; generated executables, captures and reports are saved
under ignored `artifacts/`. `-RepoRoot` and `-OutputDirectory` select alternatives.

- `BUILD_HOST_REPLAY.ps1`: compile the production barcode decoder for PC replay.
- `REPLAY_BARCODE_LOG.ps1`: replay an explicitly supplied log/result pair.
- `TEST_HELPERS.ps1`: host-only regression checks with tracked test fixtures.
- `TEST_ROBOT_USB.ps1`: capture from the identified robot; choose the procedure.
- `FLASH_ROBOT_LOCKED.ps1`: guarded build/installation helper, build-only by default.
- `INSTALL_*` and `RUN_*`: explicit legacy diagnostic profiles, not normal boot.

Check each script's parameters, profile identity and limits before hardware use.
Motion tools require explicit commands; supervised line profiles may disable
obstacle/pattern/progress stopping and do not establish whole-course autonomy.
Do not infer installed firmware or a physical PASS from historical filenames.

`source_verification.json` is an operational, historical flash-safety allowlist,
not current hardware acceptance. It deliberately fails closed when reviewed
source/manifest inputs differ; it must not be regenerated merely to bypass a
refusal. The normal build helper at `tools/build_windows.ps1` does not flash.
Supplementary bench guides and hardware evidence are retained locally only.
