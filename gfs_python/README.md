# RT-PPP GFS Python runtime

This directory is the isolated runtime boundary for GFS download and GRIB2
decoding. It is independent of `research/` and is not part of the PPP math.

## First installation

On Windows, run `bootstrap.ps1` from this directory. On Linux, run
`./bootstrap.sh`. The scripts create `.venv` locally and install only the
`eccodes` Python wheel. The `.venv` directory can be copied with the
application for a machine with the same operating system and Python ABI.

For a fully offline migration, place the matching `eccodes` wheel and its
dependencies in a local wheelhouse and install with `pip --no-index
--find-links <wheelhouse>`. Do not reuse `research/.venv`.

`bootstrap.ps1` and `bootstrap.sh` automatically prefer a sibling
`wheelhouse/` directory when it contains wheels. A portable deployment can
therefore include the application, `gfs_python/.venv`, and optionally the
wheelhouse for a clean offline rebuild on the same OS, CPU architecture and
Python ABI.

The Qt application first searches for this private environment. The optional
`RTPPP_GFS_PYTHON` environment variable can point to another dedicated Python
interpreter, and `RTPPP_GFS_WORKER` can point to `gfs_worker.py`. A system
`python` fallback is allowed for diagnostics, but deployment should use the
private `.venv`.

When running the Visual Studio Debug executable, initialize the copied runtime
in `build/vs/debug/gfs_python`; for Release, initialize
`build/vs/release/gfs_python`. The application logs the exact worker script
path and interpreter used. The worker checks `eccodes` before downloading, so
a missing environment cannot leave a newly downloaded but undecodable cache.

## Protocol

The worker reads one JSON object from stdin and writes one JSON object to
stdout. It downloads/cache-writes the selected GFS GRIB2 product, decodes the
TMP/SPFH/HGT pressure levels, performs spatial interpolation, and returns the
profile. Diagnostics are written to stderr. The C++ side retains the existing
profile integration and temporal interpolation.

If the nominal six-hour cycle returns HTTP 404, the worker retries the
previous one and then the cycle before that, preserving the requested valid
time with an equivalent forecast lead. Network, permission, timeout, and
decode errors are not silently converted into cycle fallback. The JSON
response records `selection`, `fallbackUsed`, `productCycle`, `forecastLeadSec`
and the 404 `attempts`.
