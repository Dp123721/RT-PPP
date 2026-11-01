# RT-PPP Research Analysis

`research/` is a standalone Python 3 project for scientific analysis of RT-PPP outputs. It has
no source dependency on the Qt/C++ application and can be copied to another machine unchanged.

## Scope

- SSR-corrected SP3/CLK comparison against precise products and between SSR centers.
- RINEX observation availability, satellite counts, epoch gaps, and SNR analysis.
- Compact PPP JSON analysis: independently recomputed NEU errors, convergence, and fixing rates.

The application can export structured MSM/SSR reception telemetry. Per-observation PPP residuals
remain outside the M4 scope and are still tracked in `TODO.md`. Analyses report any unavailable
telemetry explicitly instead of inferring packet loss from missing observations.

## Environment

```powershell
cd research
py -3.12 -m venv .venv
.venv\Scripts\Activate.ps1
python --version
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Python 3.11 or newer is required; Python 3.12 is the recommended Windows/PyCharm interpreter. Do
not reuse a virtual environment created by another Python version. If `research/.venv` already
reports Python 3.8, rename or remove that environment and create it again with the commands above.

`requirements.txt` contains only dependencies imported by the GUI and the three analyses. Optional
Parquet output is installed separately:

```powershell
python -m pip install -r requirements-parquet.txt
```

Configuration files are JSON, so no YAML parser is required.

## PyCharm Entry Point

Open the `research` directory as a PyCharm project. In `Settings > Project > Python Interpreter`,
create or select a Python 3.11/3.12 virtual environment; do not select the existing Python 3.8
environment. Install `requirements.txt`, then run `main.py`. No script parameters, editable package
installation, or `PYTHONPATH` setting is needed; running without arguments opens the GUI.

The same entry file supports command-line runs from PyCharm script parameters or a terminal:

```powershell
python main.py
python main.py orbit --config configs/orbit_compare.example.json
python main.py observations --config configs/observation_analysis.example.json
python main.py ppp --config configs/ppp_analysis.example.json
```

Use `gui` or `--gui` explicitly when a launch configuration must state the mode. Relative paths in
configurations continue to resolve from the configuration file, not from PyCharm's working
directory.

## Commands

```powershell
rtppp-analysis orbit --config configs/orbit_compare.example.json
rtppp-analysis observations --config configs/observation_analysis.example.json
rtppp-analysis ppp --config configs/ppp_analysis.example.json
```

The installed-package entry points remain available for terminal use:

```powershell
rtppp-analysis-gui
```

The launcher lets you create or edit a configuration interactively, browse for all input files,
manage orbit candidates or analysis datasets, set numeric options, and select an output directory.
`Validate and save` applies the same strict Schema validation used by the CLI and then selects the
new JSON file in the main window. The main window can also load an existing configuration, override
its output directory, start or stop a run, and display the CLI log. For source checkouts, prefer
`python main.py`; the module commands require package installation or an explicit module path.

Every run writes a `manifest.json`, normalized CSV tables, metrics JSON, figures, and `report.html`.
Input paths are resolved relative to the configuration file, while output paths are resolved the
same way unless absolute.

## Validation and Tests

Configurations are validated before inputs are opened. Generated `metrics.json` values are
normalized to standard JSON and validated before they are written. The canonical Draft 2020-12
contracts are installed from `schemas/` with the Python package.

Run the dependency-free test entry point from the repository root after installing the package:

```powershell
python -m compileall -q research/src
python -m unittest discover -s research/tests -v
```

On Windows, the same checks are available as a single command:

```powershell
research\run_tests.ps1
```

The contracts are strict. Unknown configuration fields, duplicate product or dataset IDs, invalid
array sizes, non-finite JSON numbers, and duplicate data keys are errors. Duplicate keys are:

- SP3: epoch, satellite, and P/V record type.
- CLK: epoch and satellite for AS records.
- RINEX OBS: epoch, satellite, and observation type. Multiple system blocks at one epoch are valid.
- Compact PPP JSON: UTC solution epoch.

## Time and Product Semantics

Each SP3/CLK input declares an explicit `time_scale` (`GPS` or `UTC`). This is required because
some locally generated files contain UTC epoch text even when a standard header advertises GPS.
The comparison also requires explicit `orbit_reference` metadata (`APC`, `COM`, or `UNKNOWN`) and
an explicit `reference_frame`, such as `IGS20`, `ITRF2020`, or `UNKNOWN`. Incompatible time scales,
incompatible APC/CoM or reference-frame declarations, and unknown reference semantics produce
warnings. RT-PPP-generated SP3/CLK calendar text is UTC even when legacy headers advertise GPS, so
candidate configurations must use `UTC`.

## Orbit and Clock Analysis

Use one precise SP3/CLK product as `reference`, then add one or more SSR-corrected SP3/CLK
products as `candidates`. Each product needs a unique ID, an SP3 path, its actual calendar time
scale, orbit reference (`APC`, `COM`, or `UNKNOWN`), and reference frame. Add a CLK path whenever
clock comparison or SISRE is required. Paths may be absolute or relative to the configuration JSON.

Start from `configs/orbit_compare.example.json`, replace the input paths and metadata, then run:

```powershell
python main.py orbit --config configs/orbit_compare.json
```

In the GUI choose **Orbit and clock**, load or create the same configuration, select exactly one
reference product and at least one candidate, then start the run. `time_scale` must describe the
timestamps actually written in the files, rather than only the nominal product convention. The
RT-PPP local writer uses `UTC`; most downloaded precise products use `GPS`.

The analysis aligns candidate epochs to the reference with bounded orbit and clock interpolation.
It reports ECEF and RAC residuals, 3D orbit error, raw clock residual, and clock residual after a
per-constellation/per-epoch median datum removal. It also reports SISRE for every common valid
orbit-and-clock sample:

```text
SISRE = sqrt((R - c*dT)^2 + (A^2 + C^2)/49)
```

`R`, `A`, and `C` are candidate-minus-reference RAC residuals in metres; `c*dT` is the
datum-removed candidate-minus-reference satellite-clock residual in metres. SISRE is `null` in
summaries or absent for samples if either orbit or clock is unavailable. It is meaningful only when
the product time scale, APC/CoM semantics, and reference frame are compatible; inspect `warnings`
in `metrics.json` before interpreting a ranking.

Outputs include `orbit_clock_residuals.csv` (per sample), `orbit_clock_summary.csv` (per system
and scope), `orbit_clock_satellite_summary.csv`, `metrics.json`, three figures, `report.html`, and
`manifest.json`. The `common` summary uses only epoch/satellite pairs available for every candidate.

SP3 position records are converted from kilometers to meters, velocity records from decimeters per
second to meters per second, and SP3 clocks from microseconds to seconds. RINEX CLK `AS` values are
already seconds. Missing SP3 positions are omitted and missing SP3 clocks remain unavailable;
non-finite record values and malformed P/V/AS records are errors. Formal SP3 header conformance is
still tracked separately because the current C++ writer can overwrite the first header lines while
updating its satellite list; record parsing therefore does not infer time semantics from that header.

Observation datasets require `station`, `rinex`, and `time_scale`. RT-PPP-generated RINEX calendar
text is UTC. Signal coverage is reported against all parsed epochs and satellites in the dataset;
it is not a packet-loss estimate. The parser supports multi-line `SYS / # / OBS TYPES` headers,
multi-line satellite records, blank observation fields, LLI/SSI characters, and event epochs.
Satellite records with no numeric values remain part of epoch and coverage counts.

Observation output includes `observation_epochs.csv`, all finite fields with LLI/SSI in
`observation_values.csv`, `signal_summary.csv`, `snr_satellite_summary.csv`, explicit event and gap
tables, and signal-coverage/SNR figures. Signal summaries distinguish value-opportunity coverage,
epoch coverage, and within-system satellite coverage.

When a dataset provides `telemetry_directory`, the analyzer reads the daily `msm_YYYYMMDD.csv`,
`ssr_YYYYMMDD.csv`, and `reception_YYYYMMDD.csv` files written by the Qt application. It writes
normalized copies plus `lock_arcs.csv`, `cycle_slips.csv`, `half_cycle_events.csv`, and
`snr_elevation_summary.csv`. Lock resets and half-cycle changes are evidence-backed events; a
large epoch gap only terminates an arc and is not called a cycle slip. Missing elevation remains
`unknown` rather than a fabricated zero. Set `options.write_parquet` to `true` to additionally
write ZSTD-compressed Parquet partitions under `telemetry_parquet/date=.../station=.../source=...`;
this requires the `full` extra (`pandas` and `pyarrow`).
The HTML report includes elevation-binned CNR, lock-arc/cycle-slip, and reception-integrity figures
when the corresponding telemetry is present.

PPP datasets require an independent three-component `reference_ecef_m` in meters. The reference
embedded by the application can be the first solution when no CRD/apriori coordinate was supplied,
so it is not accepted as scientific truth. Compact v1 inputs must be finalized `.json` files;
runtime `.json.part` files are rejected. Compact v1 stores the field named `enuM` in historical
`[N, E, U]` order. The analyzer never treats it as truth: it recomputes NEU from ECEF and the
configured reference, then reports the stored-minus-computed difference.

The three mutually exclusive reference modes are:

- Static: provide only `reference_ecef_m`.
- Linear motion: also provide `reference_velocity_ecef_mps` and `reference_epoch_utc`.
- Dynamic truth: provide `truth_trajectory`, a CSV with unique `epoch_utc,x_m,y_m,z_m` rows.

Trajectory coordinates are linearly interpolated only inside the supplied time span. Outside that
span the solution is missing, not extrapolated. The GUI exposes all reference fields, residual
inputs, convergence controls, jump thresholds, bootstrap settings, and output browsing.

Enable `Write structured MSM/SSR/PPP telemetry` in the Qt application's preparation output settings
to create `ppp_residual_YYYYMMDD.csv`. A PPP dataset can consume that directory through
`residual_directory`; use `residual_source_id` when the directory contains multiple streams. Each
row contains source, UTC epoch, satellite, signal, code/phase/iono type, prefit innovation, postfit
residual, innovation standard deviation, elevation, adopted state, and rejection reason. The final
key `(source, epoch, satellite, signal, observation_type)` must be unique. `NIS` is computed as
`(prefit_m / sigma_m)^2`; postfit normalization is retained separately for residual outlier plots.

PPP output includes `ppp_epochs.csv`, common-epoch differences, jump events, fixed segments,
reconvergence events, normalized residual tables, Schema-validated `metrics.json`, and an HTML
report. Figures cover NEU/fixed satellites, coordinate-error histograms, ZTD/wet delay/HDOP, and
residual time series, distributions, elevation dependence, and rejection counts. Metrics include
N/E/U, horizontal and 3D bias/STD/RMS/P68/P95/maximum, covariance-rotated standard deviations,
normalized coordinate errors, position NEES, residual NIS, convergence with an explicit maximum
gap, fixing ratios, paired sign tests, and deterministic bootstrap confidence intervals.

Statistics contain `count`, `total_count`, `missing_count`, and `coverage_rate` alongside the
descriptive values. Empty or non-finite results serialize as JSON `null`.

## Missing-Data Vocabulary

Structured telemetry uses the following reserved reasons: `not_transmitted`, `not_received`,
`stream_interruption`, `stale`, `iod_mismatch`, `rejected`, `invalid`, and `unknown`. The C++
exporter records received messages and evidence-backed SSR rejection reasons. A receiver cannot
distinguish a message that was never transmitted from one lost before reception, so those cases
must remain `unknown` unless upstream/provider telemetry supplies the distinction; absence alone
is never converted into a packet-loss claim.

See `TODO.md` for scientific acceptance rules and the remaining research roadmap.
