# Current Worktree Changeset Inventory

This inventory describes the intended ownership of the current dirty worktree.
It is not a commit manifest and does not claim that every pre-existing change
was created in one session.

## Main-program runtime and services

- `mainwindow.*`, `main.cpp`, and `ui/`: UI composition, command routing, and
  presentation of immutable runtime snapshots.
- `rtcmprocessingservice.*`, `rtcmdecoder.*`: off-thread RTCM buffering,
  decoding, MSM merge/sample handling, source-isolated correction state,
  preparation snapshots, corrected observations, SP3 records, and sky points.
- `pppruntimeservice.*`, `ppp_worker.*`, `ppp_engine.*`: centralized PPP
  lifecycle and the PPP calculation pipeline.
- `streamclients.*`, `settingsstore.*`: stream ingress and configuration.

## Scientific analysis package

- `research/src/rtppp_analysis/`: independent command-line and GUI pipelines
  for orbit, observation, PPP, plotting, reports, schemas, and provenance.
- `research/tests/`: synthetic pipeline, parser, schema, and metric tests.

## GNSS common behavior

- `gnss_utils.*`: GPS/UTC, MSM, SSR, GLONASS, BDS, and week-boundary handling.
- `gnss_signals.*`, `gnss_ephemeris.*`: common signal and ephemeris helpers.

## Runtime output boundary

- `outputworker.*`: the only runtime owner of output jobs and file writes.
- `sp3writer.*`, `rinexobswriter.*`, `pppjsonwriter.*`, `telemetrywriter.*`:
  serializers owned and called by `OutputWorker`.

## Build, tests, and documentation

- `RT-PPP.sln`, `RT-PPP.vcxproj*`, `RT-PPP.pro`: VS2022 and qmake manifests.
- `tests/`: GNSS time, PPP JSON, BIE, output, lifecycle, and architecture
  boundary smoke tests.
- `ARCHITECTURE.md`, `README.md`, `BIE_ALGORITHM.md`,
  `PPP_JSON_OUTPUT.md`: design and operational documentation.

## Explicitly excluded from this changeset

- Local reference source trees: read-only comparison material and libraries.
- Generated `debug/`, `release/`, `build/`, `.vs/`, and test build products.

The excluded trees must not appear in the main-program build manifests. Their
presence in `git status` is not evidence that they belong to this changeset.

## Verification

Run the complete local gate from a VS2022-capable PowerShell session:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tests\run_smoke.ps1
```

The gate builds Debug x64, compiles and executes focused C++ smoke tests,
checks architecture boundaries, and verifies natural Qt application shutdown.
